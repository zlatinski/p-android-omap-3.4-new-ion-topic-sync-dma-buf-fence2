/*
 * Copyright (C) Texas Instruments - http://www.ti.com/
 * Author: Tony Zlatinski <zlatinski@ti.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <linux/debugfs.h>
#include <linux/export.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/kref.h>
#include <linux/idr.h>
#include <linux/ion.h>
#include <linux/fdtable.h>
#include <linux/syscalls.h>
#include <linux/omap_ion.h>

#include <linux/uaccess.h>
#include <linux/anon_inodes.h>

#include <linux/dma-buf.h>
#include <omap_dmm_tiler.h>
#include <video/omap_gfx_buf.h>

#define DEBUG_OMAPGFX_BUF
//#define DEBUG_OMAPGFX_BUF_DUMP_PARAMS

#ifdef DEBUG_OMAPGFX_BUF
#define gfx_buf_info printk
#else
#define gfx_buf_info pr_info
#endif

static DEFINE_IDR(omap_gfx_buf_names_idr);
static DEFINE_SPINLOCK(omap_gfx_buf_idr_lock);
static unsigned int omap_gfx_buf_next_idr_name;

static unsigned int init_called;

static int omap_gfx_buf_free(omap_gfx_buf_t *gfx_buf);
static int omap_gfx_buf_release(struct inode *inode, struct file *file);

static void omap_gfx_buf_free_planes(omap_gfx_buf_t *gfx_buf);
static int omap_gfx_buf_allocate_planes(void* buf_mgr_cxt,
                                        omap_gfx_buf_t *gfx_buf,
                                        omap_gfx_buf_req_t* req);

/* Obtain a handle for the buffer */
static int omap_gfx_buf_get_name(omap_gfx_buf_t *gfx_buf)
{
        int ret;
        unsigned long flags;

        do {
                if (idr_pre_get(&omap_gfx_buf_names_idr, GFP_KERNEL) == 0) {
                        ret = -ENOMEM;
                        return ret;
                }

                spin_lock_irqsave(&omap_gfx_buf_idr_lock, flags);

                /* Deal with the name roll-over.
                 * 0 is an invalid name for gfx_buf */
                if(omap_gfx_buf_next_idr_name == 0)
                        omap_gfx_buf_next_idr_name++;

                /* Register the object and get a name */
                ret = idr_get_new_above(&omap_gfx_buf_names_idr, gfx_buf,
                                        omap_gfx_buf_next_idr_name++,
                                        &gfx_buf->name);

                spin_unlock_irqrestore(&omap_gfx_buf_idr_lock, flags);

                if ((ret != -EAGAIN) && (ret != 0))
                        return ret;

        } while(ret == -EAGAIN);

        return ret;
}

/* Release the handle of the buffer */
static int omap_gfx_buf_release_name(omap_gfx_buf_t *gfx_buf)
{
        int old_name = 0;
        unsigned long flags;

        /* Remove any name for this object */
        spin_lock_irqsave(&omap_gfx_buf_idr_lock, flags);
        if (gfx_buf->name) {
                old_name = gfx_buf->name;
                idr_remove(&omap_gfx_buf_names_idr, gfx_buf->name);
                gfx_buf->name = 0;
        }
        spin_unlock_irqrestore(&omap_gfx_buf_idr_lock, flags);

        return old_name;
}

static const char* omap_gfx_buf_mem_flag_to_str(unsigned int mem_flag)
{
        switch(mem_flag) {
                /* Allocation Information flags */
        case TI_MEM_TYPE_SYSTEM:
                return "SYSTEM";
        case TI_MEM_TYPE_CONTIG:
                return "CONTIG";
        case TI_MEM_TYPE_SECURE:
                return "SECURE";
        case TI_MEM_TYPE_TILER_8BIT	:
                return "TILER_8BIT";
        case TI_MEM_TYPE_TILER_16BIT:
                return "TILER_16BIT";
        case TI_MEM_TYPE_TILER_32BIT:
                return "TILER_32BIT";
        case TI_MEM_TYPE_TILER_PAGE:
                return "TILER_PAGE";
        case TI_MEM_TYPE_MULTI_PLANAR:
                return "MULTI_PLANAR";
        case TI_MEM_TYPE_INTERLEAVED:
                return "INTERLEAVED";
        case TI_MEM_TYPE_2_1_HOR_SUBSAMPLED:
                return "2_1_HOR_SUBSAMPLED";
        case TI_MEM_TYPE_2_1_VER_SUBSAMPLED:
                return "2_1_VER_SUBSAMPLED";
        case TI_MEM_TYPE_FB_VRAM:
                return "FB_VRAM";
        case TI_MEM_TYPE_VRAM: /* Prefer FAST VRAM (not DRAM) */
                return "VRAM";
                /* Can migrate the physical memory from VRAM to DRAM and vice-versa */
        case TI_MEM_TYPE_PHYS_MIGRATE:
                return "PHYS_MIGRATE";
        case TI_MEM_TYPE_READ:
                return "READ";
        case TI_MEM_TYPE_WRITE:
                return "WRITE";
        case TI_MEM_TYPE_CACHED:
                return "CACHED";
        case TI_MEM_TYPE_WC: /* WRITECOMBINE Mode */
                return "WC";
        case TI_MEM_TYPE_KERNEL_ONLY: /* Can not map/access to/from CPU */
                return "KERNEL_ONLY";
        case TI_MEM_TYPE_SINGLE_PROCESS: /* Can't export to another process */
                return "SINGLE_PROCESS";
        case TI_MEM_TYPE_MAP_CPU_PAGEABLE: /* Supports pageable memory */
                return "MAP_CPU_PAGEABLE";
        case TI_MEM_TYPE_ZERO_INIT:
                return "ZERO_INIT";
        default:
                return "UNKNOWN";
        }
}

static void omap_gfx_buf_mem_flag_dump(unsigned int ti_mem_flags)
{
        unsigned int mask = 1;

        gfx_buf_info("\tMemory Flags:\n");
        do {
                unsigned int flag = (ti_mem_flags & mask);

                if(flag) {
                        gfx_buf_info("\t\t%s \t\t(0x%08x),\n",
                                     omap_gfx_buf_mem_flag_to_str(flag), flag);
                }
                mask = mask << 1;
        } while(mask);
        gfx_buf_info("\n");
}

typedef struct gfx_buf_dump_info
{
        struct seq_file *m;
        size_t total_size;
}gfx_buf_dump_info_t;

/* Dump buffer info */
static int
omap_gfx_buf_dump_info(int id, void *ptr, void *data)
{
        omap_gfx_buf_t *gfx_buf = ptr;
        gfx_buf_dump_info_t* p_dump_info = data;
        struct seq_file *m = p_dump_info->m;
        unsigned i;

        seq_printf(m, "Buffer name %d", gfx_buf->name);
        seq_printf(m, " with format 0x%08x ", gfx_buf->pixel_format);
        seq_printf(m, " and num planes %d\n", gfx_buf->num_planes);

        for(i = 0; i < gfx_buf->num_planes; i++) {
                unsigned int mask = 1;

                seq_printf(m, "\t%d). size: %d, (w %d x h %d) @ %d bpp, "
                        "flags 0x%08x\n"
                        "\t\tstride %d pixels, offset %d bytes, "
                        "alignment %d bytes\n",
                        i, gfx_buf->planes[i].params.size_bytes,
                        gfx_buf->planes[i].params.width,
                        gfx_buf->planes[i].params.height,
                        gfx_buf->planes[i].params.bpp,
                        gfx_buf->planes[i].params.mem_flags,
                        gfx_buf->planes[i].params.stride_pixels,
                        gfx_buf->planes[i].params.offset_bytes,
                        gfx_buf->planes[i].params.align_bytes);

                seq_printf(m, "\tMemory Flags: ");
                do {
                        unsigned int flag =
                                (gfx_buf->planes[i].params.mem_flags & mask);

                        if(flag) {
                                seq_printf(m, "%s (0x%08x), ",
                                  omap_gfx_buf_mem_flag_to_str(flag), flag);
                        }
                        mask = mask << 1;
                } while(mask);
                seq_printf(m, "\n");

                if(gfx_buf->planes[i].dmabuf)
                    p_dump_info->total_size += gfx_buf->planes[i].dmabuf->size;
        }

        return 0;
}

/**
 * Dump all current buffers info
 */
static int
omap_gfx_buf_debugfs_show(struct seq_file *m, void *unused)
{
        gfx_buf_dump_info_t dump_info;

        dump_info.m = m;
        dump_info.total_size = 0;

        seq_printf(m, "Live Buffers:\n");

        idr_for_each(&omap_gfx_buf_names_idr,
                     &omap_gfx_buf_dump_info, &dump_info);

        seq_printf(m, "*** Total Buffers Size %d bytes ***\n",
                (unsigned)dump_info.total_size);
        return 0;
}

/* Force shutdown release of a gfx buffer object */
static int
omap_gfx_buf_shutdown_release(int id, void *ptr, void *data)
{
        omap_gfx_buf_t *gfx_buf = ptr;
//	unsigned int priv_data = *(unsigned int*)data;

        omap_gfx_buf_free(gfx_buf);

        return 0;
}

/* Force shutdown release of all the buffers */
static
int omap_gfx_buf_release_all_names(void)
{
        unsigned int priv_data = 0;

        idr_for_each(&omap_gfx_buf_names_idr,
                     &omap_gfx_buf_shutdown_release, &priv_data);

        idr_remove_all(&omap_gfx_buf_names_idr);
        idr_destroy(&omap_gfx_buf_names_idr);

        return 0;
}

static long omap_gfx_buf_ioctl_buf_info(omap_gfx_buf_t *gfx_buf,
                                        unsigned long arg)
{
        omap_gfx_buf_info_t omap_gfx_buf_info;
        __u32 len = sizeof(omap_gfx_buf_info_t);
        int ret;
        unsigned int i;

        omap_gfx_buf_info.pixel_format = gfx_buf->pixel_format;
        omap_gfx_buf_info.name = gfx_buf->name;
        omap_gfx_buf_info.num_planes = gfx_buf->num_planes;

        for(i = 0; i < gfx_buf->num_planes; i++) {
                memcpy(&omap_gfx_buf_info.plane_params[i],
                       &gfx_buf->planes[i].params,
                       sizeof(omap_gfx_buf_info.plane_params[i]));
        }

        if (copy_to_user((void __user *)arg, &omap_gfx_buf_info, len))
                ret = -EFAULT;
        else
                ret = 0;

        return ret;
}

static long omap_gfx_buf_ioctl_sync(omap_gfx_buf_t *gfx_buf, unsigned long arg)
{
        __s32 value;

        if (copy_from_user(&value, (void __user *)arg, sizeof(value)))
                return -EFAULT;

        pr_err("%s: SYNC not implemented yet for dma-buf\n",
               __func__);

        return -ENOTTY;
}

static long omap_gfx_buf_ioctl(struct file *file, unsigned int cmd,
                               unsigned long arg)
{
        omap_gfx_buf_t *gfx_buf = file->private_data;

        switch (cmd) {

        case OMAP_GFX_BUF_IOC_BUF_PARAMS:
                return omap_gfx_buf_ioctl_buf_info(gfx_buf, arg);

        case OMAP_GFX_BUF_IOC_SYNC:
                return omap_gfx_buf_ioctl_sync(gfx_buf, arg);

        default:
                return -ENOTTY;
        }
}

/* omap_gfx_buf file ops */
static const struct file_operations omap_gfx_buf_fops = {
        .release = omap_gfx_buf_release,
        .unlocked_ioctl = omap_gfx_buf_ioctl,
};

/*
 * is_omap_gfx_buf_file - Check if struct file*
 * is associated with omap_gfx_buf
 */
static inline int is_omap_gfx_buf_file(struct file *file)
{
        return file->f_op == &omap_gfx_buf_fops;
}

static size_t get_omap_gfx_buf_size(unsigned int num_planes)
{
        /* omap_gfx_buf_t has one plane by default */
        return (sizeof(omap_gfx_buf_t) +
                ((num_planes - 1) * sizeof(omap_gfx_buf_plane_t)));
}

/* Create a new omap_gfx_buf */
int omap_gfx_buf_create(void* buf_mgr_cxt,
                        omap_gfx_buf_req_t* req)
{
        omap_gfx_buf_t *gfx_buf;
        int ret;
        unsigned i, num_planes = 0;


#ifdef DEBUG_OMAPGFX_BUF_DUMP_PARAMS
        {
                gfx_buf_info("=> Client Requested buffer with pixel "
                        "format 0x%08x\n", req->pixel_format);

                for(i = 0; i < TI_MAX_SUB_ALLOCS; i++) {
                        if(!req->planes[i].params.mem_flags)
                                break;

                        gfx_buf_info("%d). size: %d, (w %d x h %d) @ %d bpp,"
                                "flags 0x%08x\n"
                                     "\t\tstride %d pixels, offset %d bytes,"
                                     "alignment %d bytes\n",
                                     i, req->planes[i].params.size_bytes,
                                     req->planes[i].params.width,
                                     req->planes[i].params.height,
                                     req->planes[i].params.bpp,
                                     req->planes[i].params.mem_flags,
                                     req->planes[i].params.stride_pixels,
                                     req->planes[i].params.offset_bytes,
                                     req->planes[i].params.align_bytes);

                        omap_gfx_buf_mem_flag_dump(req->planes[i].params.mem_flags);
                }
        }
#endif

        /* How many planes do we need */
        for(i = 0; i < TI_MAX_SUB_ALLOCS; i++) {
                if(!req->planes[i].params.mem_flags)
                        break;

                num_planes++;
        }

        /* At least one valid plane must exist */
        if(!num_planes) {
                pr_err("%s: Invalid all planes mem_flags == 0\n",
                       __func__);
                return -EINVAL;
        }

        gfx_buf = kzalloc(get_omap_gfx_buf_size(num_planes), GFP_KERNEL);
        if (gfx_buf == NULL) {
                pr_err("%s: Can't allocate object - out-of-memory\n",
                       __func__);
                ret = -ENOMEM;
                return ret;
        }

        gfx_buf->num_planes = num_planes;
        mutex_init(&gfx_buf->lock);

        for(i = 0; i < TI_MAX_SUB_ALLOCS; i++)
                req->planes[i].export_fd = -1;

        req->sync_fd = -1;

        /* Allocate the buffers first */
        ret = omap_gfx_buf_allocate_planes(buf_mgr_cxt, gfx_buf, req);
        if(IS_ERR_VALUE(ret)) {
                pr_err("%s: Can't allocate buffer planes - %d\n",
                       __func__, ret);
                goto err;
        }

        /* Then get the gfx buffer a name */
        ret = omap_gfx_buf_get_name(gfx_buf);
        if(IS_ERR_VALUE(ret)) {
                pr_err("%s: Can't allocate buffer's common name - %d\n",
                       __func__, ret);
                omap_gfx_buf_free_planes(gfx_buf);
                goto err;
        }

        /* Get a Fd for the user */
        req->sync_fd = anon_inode_getfd("omap_gfx_buf", &omap_gfx_buf_fops,
                                        gfx_buf, O_CLOEXEC);
        if (IS_ERR_VALUE(req->sync_fd)) {
                pr_err("%s: Can't allocate buffer's inode & fd - %d\n",
                       __func__, ret);
                omap_gfx_buf_free_planes(gfx_buf);
                omap_gfx_buf_release_name(gfx_buf);
                ret = -ENOMEM;
                goto err;
        }

        /* and a file inode */
        gfx_buf->file = fget(req->sync_fd);
        /* The sync_fd should have a valid file,
         * associated with it.
         */
        BUG_ON(!gfx_buf->file);

        /* Need only one reference for the user,
         * so when the user closes the fd,
         * the gfx_buf goes away
         */
        fput(gfx_buf->file);

        /* Send the gfx_buf name back to client */
        req->name = gfx_buf->name;

#ifdef DEBUG_OMAPGFX_BUF_DUMP_PARAMS
        {
                gfx_buf_info("<= Return: gfx_buf buffer parameters with "
                        "pixel format 0x%08x\n"
                             "\t\t ->  name %d, sync_fd %d\n",
                             req->pixel_format,
                             (unsigned)req->name, req->sync_fd);

                for(i = 0; i < TI_MAX_SUB_ALLOCS; i++) {
                        if(!req->planes[i].params.mem_flags)
                                break;

                        gfx_buf_info("%d). size: %d, (w %d x h %d) @ %d bpp, "
                                "flags 0x%08x\n"
                                     "\t\tstride %d pixels, offset %d bytes,"
                                     " alignment %d bytes"
                                     " export_fd %d\n",
                                     i, req->planes[i].params.size_bytes,
                                     req->planes[i].params.width,
                                     req->planes[i].params.height,
                                     req->planes[i].params.bpp,
                                     req->planes[i].params.mem_flags,
                                     req->planes[i].params.stride_pixels,
                                     req->planes[i].params.offset_bytes,
                                     req->planes[i].params.align_bytes,
                                     req->planes[i].export_fd);

                        omap_gfx_buf_mem_flag_dump(
                                req->planes[i].params.mem_flags);
                }
        }
#endif

        return ret;

err:
        kfree(gfx_buf);
        return ret;
}
EXPORT_SYMBOL(omap_gfx_buf_create);

/* Obtains a omap_gfx_buf object from a fd
 * increments the object reference count */
omap_gfx_buf_t *omap_gfx_buf_fdget(int fd)
{
        struct file *file = fget(fd);

        if (file == NULL)
                return NULL;

        if (!is_omap_gfx_buf_file(file)) {
                pr_err("%s: Invalid buffer inode requested for a fd: %d\n",
                       __func__, fd);
                goto err;
        }

        return file->private_data;

err:
        fput(file);
        return NULL;
}
EXPORT_SYMBOL(omap_gfx_buf_fdget);

/* Releases an omap_gfx_buf object */
void omap_gfx_buf_put(omap_gfx_buf_t *gfx_buf)
{
        fput(gfx_buf->file);
}
EXPORT_SYMBOL(omap_gfx_buf_put);

/* Installs omap_gfx_buf object to a new fd */
void omap_gfx_buf_install(omap_gfx_buf_t *gfx_buf, int fd)
{
        fd_install(fd, gfx_buf->file);
}
EXPORT_SYMBOL(omap_gfx_buf_install);

/* Obtains a new fd and installs omap_gfx_buf object to it */
int omap_gfx_buf_get_fd(omap_gfx_buf_t *gfx_buf, int flags)
{
        int error, fd;
        struct files_struct * files;
        struct fdtable * fdt;

        if (!gfx_buf || !gfx_buf->file) {
                pr_err("%s: Can't get fd for invalid gfx_buf: %p\n",
                       __func__, gfx_buf);
                return -EINVAL;
        }
        error = get_unused_fd();
        if (error < 0)
                return error;
        fd = error;

        files = current->files;
        spin_lock(&files->file_lock);
        fdt = files_fdtable(files);
        __set_close_on_exec(fd, fdt);
        spin_unlock(&files->file_lock);

        fd_install(fd, gfx_buf->file);

        return fd;
}
EXPORT_SYMBOL_GPL(omap_gfx_buf_get_fd);

/* Frees omap_gfx_buf object */
static int omap_gfx_buf_free(omap_gfx_buf_t *gfx_buf)
{
#ifdef DEBUG_OMAPGFX_BUF_DUMP_PARAMS
        {
                unsigned i;

                gfx_buf_info("Freeing gfx_buf with with name: %d, format: 0x%08x "
                "and num_planes %d\n",
                gfx_buf->name, gfx_buf->pixel_format, gfx_buf->num_planes);

                for(i = 0; i < gfx_buf->num_planes; i++) {
                        gfx_buf_info("%d). size: %d, (w %d x h %d) @ %d bpp, "
                                "flags 0x%08x\n"
                        "\t\t stride %d pixels, offset %d bytes, "
                        "alignment %d bytes"
                        " dmabuf %p\n",
                        i, gfx_buf->planes[i].params.size_bytes,
                        gfx_buf->planes[i].params.width,
                        gfx_buf->planes[i].params.height,
                        gfx_buf->planes[i].params.bpp,
                        gfx_buf->planes[i].params.mem_flags,
                        gfx_buf->planes[i].params.stride_pixels,
                        gfx_buf->planes[i].params.offset_bytes,
                        gfx_buf->planes[i].params.align_bytes,
                        gfx_buf->planes[i].dmabuf);

                        omap_gfx_buf_mem_flag_dump(
                                gfx_buf->planes[i].params.mem_flags);
                }
        }
#endif

        if(omap_gfx_buf_release_name(gfx_buf)) {
                mutex_lock(&gfx_buf->lock);

                omap_gfx_buf_free_planes(gfx_buf);

                mutex_unlock(&gfx_buf->lock);

                kfree(gfx_buf);

                return 0;
        }

        pr_err("%s: Trying to free invalid gfx_buf: %p\n",
               __func__, gfx_buf);
        return -EBADF;
}

/* Releases omap_gfx_buf object on refcount going down to 0 */
static int omap_gfx_buf_release(struct inode *inode, struct file *file)
{
        omap_gfx_buf_t *gfx_buf;

        if (!is_omap_gfx_buf_file(file)) {
                pr_err("%s: File %p does not belong to gfx_buf class\n",
                       __func__, file);
                return -EINVAL;
        }

        gfx_buf = file->private_data;

        return omap_gfx_buf_free(gfx_buf);
}

/* Get ION client from buf_mgr_cxt (buffer manager context) */
static struct ion_client* omap_gfx_buf_mgr_get_ion(void* buf_mgr_cxt) {
        struct ion_client* client = (struct ion_client*)buf_mgr_cxt;

        return client;
}

/* Releases all the buffers of gfx_buf */
static void omap_gfx_buf_free_planes(omap_gfx_buf_t *gfx_buf)
{
        unsigned i;
        for(i = 0; i < gfx_buf->num_planes ; i++) {
                if(gfx_buf->planes[i].dmabuf) {
                        dma_buf_put(gfx_buf->planes[i].dmabuf);
                        gfx_buf->planes[i].dmabuf = NULL;
                }
        }
}

/* Allocate buffers for gfx_buf */
static int omap_gfx_buf_allocate_planes(void* buf_mgr_cxt,
                                        omap_gfx_buf_t *gfx_buf,
                                        omap_gfx_buf_req_t* req)
{
        struct ion_client* client = omap_gfx_buf_mgr_get_ion(buf_mgr_cxt);
        unsigned i;
        int ret = 0;

        for(i = 0; i < gfx_buf->num_planes; i++) {
                size_t len = 0;
                unsigned int flags = 0;
                struct ion_handle* handle = NULL;
                __u16 stride_pixels = req->planes[i].params.stride_pixels;
                __u32 offset_bytes = 0;

                if(req->planes[i].params.mem_flags == 0)
                        break; /* Done */

                /* Cache will be ignored by tiler 2D */
                if(req->planes[i].params.mem_flags & TI_MEM_TYPE_CACHED)
                        flags |= ION_FLAG_CACHED;

                if(!(req->planes[i].params.mem_flags & TI_MEM_TYPE_MAP_CPU_PAGEABLE))
                        flags |= ION_FLAG_CACHED_NEEDS_SYNC;

                /* Get the bufer len */
                len = (req->planes[i].params.stride_pixels *
                       req->planes[i].params.bpp *
                       req->planes[i].params.height) >> 3;

                /*Align the buffer now */
                len = ALIGN(len, req->planes[i].params.align_bytes);

                /* Is that special heap of the Framebuffer's VRAM ? */
                if(req->planes[i].params.mem_flags & TI_MEM_TYPE_FB_VRAM) {
                        /* TODO: Hook this up to the real FB_VRAM as
                         * another ION heap */
                        pr_err("%s: Allocating FAKE FB_VRAM with name %d"
                               " ( w %d x h %d ) @ %d bpp and size: %d \n",
                               __func__, gfx_buf->name,
                               req->planes[i].params.width,
                               req->planes[i].params.height,
                               req->planes[i].params.bpp,
                               len);

                        /* TODO: This bellow will eventually go away after we have
                         * the ION FB_VRAM hook implemented.
                         */
                        req->planes[i].params.stride_pixels = stride_pixels;
                        req->planes[i].params.offset_bytes = offset_bytes;
                        req->planes[i].params.size_bytes = len;

                        memcpy(&gfx_buf->planes[i].params, &req->planes[i].params,
                               sizeof(gfx_buf->planes[i].params));

                        break;

                        /* TODO: End of the code above to go away with ION FB_VRAM */

                }/* Is that a tiler allocation ? */
                else if(req->planes[i].params.mem_flags & TI_MEM_TYPE_TILER) {
                        struct omap_ion_tiler_alloc_data tiler_alloc;
                        memset(&tiler_alloc, 0x00, sizeof(tiler_alloc));

                        tiler_alloc.h = req->planes[i].params.height;
                        tiler_alloc.w = req->planes[i].params.width;
                        tiler_alloc.flags = flags;

                        if(req->planes[i].params.mem_flags & TI_MEM_TYPE_TILER_PAGE) {
                                tiler_alloc.fmt = TILFMT_PAGE;
                                tiler_alloc.h = 1;
                                tiler_alloc.w = len;
                        } else if(req->planes[i].params.mem_flags &
                                        TI_MEM_TYPE_TILER_8BIT) {
                                tiler_alloc.fmt = TILFMT_8BIT;
                        } else if(req->planes[i].params.mem_flags &
                                        TI_MEM_TYPE_TILER_16BIT) {
                                tiler_alloc.fmt = TILFMT_16BIT;
                        } else if(req->planes[i].params.mem_flags &
                                        TI_MEM_TYPE_TILER_32BIT) {
                                tiler_alloc.fmt = TILFMT_32BIT;
                        }

                        ret = omap_ion_tiler_alloc(client, &tiler_alloc);

                        if(!IS_ERR_VALUE(ret)) {
                                handle = tiler_alloc.handle;
                                if(!(req->planes[i].params.mem_flags &
                                                TI_MEM_TYPE_TILER_PAGE)) {
                                        unsigned int bytes_per_pixel =
                                                (req->planes[i].params.bpp >> 3);
                                        /* Get the 2D buffer stride in pixels */
                                        if(bytes_per_pixel == 0)
                                                bytes_per_pixel = 1;

                                        stride_pixels = tiler_alloc.stride /
                                                bytes_per_pixel;
                                }
                                offset_bytes = (__u32)tiler_alloc.offset;
                        } else {
                                pr_err("%s: Could not allocate tiler memory "
                                        "for gfx_buf: %p\n",
                                       __func__, gfx_buf);
                                omap_gfx_buf_mem_flag_dump(
                                       req->planes[i].params.mem_flags);
                        }
                } /* Is that a system-contig. allocation ? */
                else if(req->planes[i].params.mem_flags & TI_MEM_TYPE_CONTIG) {
                        handle = ion_alloc(client, len,
                                req->planes[i].params.align_bytes,
                                (1 << ION_HEAP_TYPE_SYSTEM_CONTIG), flags);

                        if(IS_ERR_OR_NULL(handle)) {
                                pr_err("%s: Could not allocate system contig. "
                                        "memory for "
                                        "gfx_buf: %p\n", __func__, gfx_buf);
                                omap_gfx_buf_mem_flag_dump(
                                        req->planes[i].params.mem_flags);
                                ret = -ENODEV;
                        }

                }/* Or a regular system memory allocation ? */
                else { /* if(req->planes[i].params.mem_flags & TI_MEM_TYPE_SYSTEM) */
                        handle = ion_alloc(client, len,
                                req->planes[i].params.align_bytes,
                                (1 << ION_HEAP_TYPE_SYSTEM), flags);

                        if(IS_ERR_OR_NULL(handle)) {
                                pr_err("%s: Could not allocate regular "
                                        "system memory for "
                                        "gfx_buf: %p\n", __func__, gfx_buf);
                                omap_gfx_buf_mem_flag_dump(
                                        req->planes[i].params.mem_flags);
                                ret = -ENODEV;
                        }
                }

                if(IS_ERR_VALUE(ret)) {
                        break;
                }

                /* Get the dma_buf representing the handle */
                /* This fd holds one reference for the ION handle.
                 * It goes to the client */
                req->planes[i].export_fd = ion_share_dma_buf(client, handle);
                if(IS_ERR_VALUE(req->planes[i].export_fd)) {
                        pr_err("%s: Could not export ION handle to dma-buf for "
                               "gfx_buf: %p\n", __func__, gfx_buf);
                        ret = req->planes[i].export_fd;
                        break;
                }

                /* Obtain the dmabuf from the fd, taking an extra reference
                 * to it.
                 * This extra reference of dmabuf ensures the buffer does not
                 * go away, while in use by gfx_buf clients during rendering
                 * and sync. */
                gfx_buf->planes[i].dmabuf = dma_buf_get(
                        req->planes[i].export_fd);
                /* dmabuf should never be invalid if we got an fd above */
                BUG_ON(gfx_buf->planes[i].dmabuf == NULL);
                if(gfx_buf->planes[i].dmabuf == NULL) {
                        ret = -ENODEV;
                        break;
                }

                /* Got dma_buf, we do not need the handle anymore */
                ion_free(client, handle);

                req->planes[i].params.stride_pixels = stride_pixels;
                req->planes[i].params.offset_bytes = offset_bytes;
                req->planes[i].params.size_bytes =
                        gfx_buf->planes[i].dmabuf->size;

                memcpy(&gfx_buf->planes[i].params, &req->planes[i].params,
                       sizeof(gfx_buf->planes[i].params));
        }

        if(IS_ERR_VALUE(ret)) {
                /* Clean-up the user handles first */
                for(i = 0; i < gfx_buf->num_planes; i++) {
                        if(req->planes[i].export_fd > 0) {
                                sys_close(req->planes[i].export_fd);
                                req->planes[i].export_fd = -1;
                        }
                }

                /* Then release the buffers */
                omap_gfx_buf_free_planes(gfx_buf);
        } else {
                /* Save the memory context and pixel format */
                gfx_buf->buf_mgr_cxt = buf_mgr_cxt;
                gfx_buf->pixel_format = req->pixel_format;
                /* Return the number of planes allocated */
                req->num_planes = gfx_buf->num_planes;
        }

        return ret;
}

#ifdef CONFIG_DEBUG_FS
static int omap_gfx_buf_debugfs_open(struct inode *inode, struct file *file)
{
        return single_open(file, omap_gfx_buf_debugfs_show, inode->i_private);
}

static const struct file_operations omap_gfx_buf_debugfs_fops = {
        .open           = omap_gfx_buf_debugfs_open,
        .read           = seq_read,
        .llseek         = seq_lseek,
        .release        = single_release,
};

static int omap_gfx_buf_debugfs_init(void)
{
        debugfs_create_file("omap_gfx_buf", S_IRUGO, NULL, NULL,
                &omap_gfx_buf_debugfs_fops);
        return 0;
}

#define DUMP_CHUNK 256
static char sync_dump_buf[64 * 1024];
static void omap_gfx_buf_dump(void)
{
        struct seq_file s = {
                .buf = sync_dump_buf,
                .size = sizeof(sync_dump_buf) - 1,
        };
        int i;

        omap_gfx_buf_debugfs_show(&s, NULL);

        for (i = 0; i < s.count; i += DUMP_CHUNK) {
                if ((s.count - i) > DUMP_CHUNK) {
                        char c = s.buf[i + DUMP_CHUNK];
                        s.buf[i + DUMP_CHUNK] = 0;
                        gfx_buf_info("%s", s.buf + i);
                        s.buf[i + DUMP_CHUNK] = c;
                } else {
                        s.buf[s.count] = 0;
                        gfx_buf_info("%s", s.buf + i);
                }
        }
}
#else
static void sync_dump(void)
{
}
#endif

void omap_gfx_buf_mgr_shutdown(void)
{
        omap_gfx_buf_release_all_names();

        omap_gfx_buf_dump();

        init_called = 0;
}

int omap_gfx_buf_mgr_init(void)
{
        if(init_called)
                return 0;

        init_called = 1;

        return omap_gfx_buf_debugfs_init();
}

