/*
 * Copyright 2004-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file ipu_vdi_enc.c
 *
 * @brief IPU Use case for VDI-ENC
 *
 * @ingroup IPU
 */

#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/ipu.h>
#include "mxc_v4l2_capture.h"

#ifdef CAMERA_DBG
	#define CAMERA_TRACE(x) (printk)x
#else
	#define CAMERA_TRACE(x)
#endif


/*
 * Function definitions
 */
static void vdi_work_func(struct work_struct *work)
{

	int err;
	cam_data *cam = (cam_data *)  container_of(work, cam_data, vdi_work);

	if(!cam->vdi_enc_first_frame) {
		err = ipu_update_channel_buffer(cam->ipu, MEM_VDI_PRP_VF_MEM_P,
			IPU_INPUT_BUFFER, 0,
			cam->vdi_enc_bufs[(cam->ping_pong_vdi + 2) %3] +
			cam->v2f.fmt.pix.bytesperline);

		if (err != 0) {
			ipu_clear_buffer_ready(cam->ipu, MEM_VDI_PRP_VF_MEM_P,
					       IPU_INPUT_BUFFER, 0);
			err = ipu_update_channel_buffer(cam->ipu,
				MEM_VDI_PRP_VF_MEM_P, IPU_INPUT_BUFFER, 0,
				cam->vdi_enc_bufs[(cam->ping_pong_vdi + 2) %3] +
				cam->v2f.fmt.pix.bytesperline);

			if (err != 0) {
				pr_err("ERROR: v4l2 capture: fail to update vdi P buffer");
				return;
			}
		}

		err = ipu_update_channel_buffer(cam->ipu, MEM_VDI_PRP_VF_MEM,
					IPU_INPUT_BUFFER, 0,
					cam->vdi_enc_bufs[cam->ping_pong_vdi]);

		if (err != 0) {
			ipu_clear_buffer_ready(cam->ipu, MEM_VDI_PRP_VF_MEM,
					       IPU_INPUT_BUFFER, 0);
			err = ipu_update_channel_buffer(cam->ipu,
					MEM_VDI_PRP_VF_MEM, IPU_INPUT_BUFFER, 0,
					cam->vdi_enc_bufs[cam->ping_pong_vdi]);

			if (err != 0) {
				pr_err("ERROR: v4l2 capture: fail to update vdi buffer");
				return;
			}
		}

		err = ipu_update_channel_buffer(cam->ipu, MEM_VDI_PRP_VF_MEM_N,
					IPU_INPUT_BUFFER, 0,
					cam->vdi_enc_bufs[cam->ping_pong_vdi]+
					cam->v2f.fmt.pix.bytesperline);

		if (err != 0) {
			ipu_clear_buffer_ready(cam->ipu, MEM_VDI_PRP_VF_MEM_N,
					       IPU_INPUT_BUFFER, 0);
			err = ipu_update_channel_buffer(cam->ipu,
					MEM_VDI_PRP_VF_MEM_N,
					IPU_INPUT_BUFFER, 0,
					cam->vdi_enc_bufs[cam->ping_pong_vdi] +
					cam->v2f.fmt.pix.bytesperline);

			if (err != 0) {
				pr_err("ERROR: v4l2 capture: fail to update vdi N buffer");
				return;
			}
		}
		ipu_select_multi_vdi_buffer(cam->ipu, 0);
	}

	cam->vdi_enc_first_frame = 0;
	cam->ping_pong_vdi = (cam->ping_pong_vdi + 1)%3;
	/* Point the CSI output to the next buffer */
	err = ipu_update_channel_buffer(cam->ipu, CSI_MEM, IPU_OUTPUT_BUFFER, 0,
					cam->vdi_enc_bufs[cam->ping_pong_vdi]);

	if (err != 0) {
		ipu_clear_buffer_ready(cam->ipu, CSI_MEM, IPU_OUTPUT_BUFFER, 0);
		err = ipu_update_channel_buffer(cam->ipu, CSI_MEM,
					IPU_OUTPUT_BUFFER, 0,
					cam->vdi_enc_bufs[cam->ping_pong_vdi]);

		if (err != 0) {
			pr_err("ERROR: v4l2 capture: fail to update CSI buffer");
			return;
		}
	}
	ipu_select_buffer(cam->ipu, CSI_MEM, IPU_OUTPUT_BUFFER, 0);
}

/*!
 * IPU ENC callback function.
 *
 * @param irq       int irq line
 * @param dev_id    void * device id
 *
 * @return status   IRQ_HANDLED for handled
 */
static irqreturn_t vdi_enc_callback(int irq, void *dev_id)
{
	cam_data *cam = (cam_data *) dev_id;

	if (cam->enc_callback)
		cam->enc_callback(irq, dev_id);

	return IRQ_HANDLED;
}

/*!
 * IPU ENC callback function for CSI->Mem.
 *
 * @param irq       int irq line
 * @param dev_id    void * device id
 *
 * @return status   IRQ_HANDLED for handled
 */
static irqreturn_t csi_enc_callback(int irq, void *dev_id)
{
	cam_data *cam = (cam_data *) dev_id;

	if (queue_work(cam->vdi_wq, &cam->vdi_work) == 0)
		pr_debug( "vdi work was in queue already\n");

	return IRQ_HANDLED;
}

/*!
 * VDI-ENC enable channel setup function
 *
 * @param cam       struct cam_data * mxc capture instance
 *
 * @return  status
 */
static int vdi_enc_setup(cam_data *cam)
{
	ipu_channel_params_t csi;
	ipu_channel_params_t enc;
	int err = 0;
	dma_addr_t dummy = 0xdeadbeaf;

	CAMERA_TRACE("In vdi_enc_setup\n");
	if (!cam) {
		printk(KERN_ERR "cam private is NULL\n");
		return -ENXIO;
	}
	memset(&enc, 0, sizeof(ipu_channel_params_t));
	memset(&csi, 0, sizeof(ipu_channel_params_t));

	/* Setup CSI to memory parameters */
	csi.csi_mem.csi = cam->csi;

	switch (ipu_csi_get_sensor_protocol(cam->ipu, cam->csi)) {
		case IPU_CSI_CLK_MODE_CCIR656_INTERLACED:
		case IPU_CSI_CLK_MODE_CCIR1120_INTERLACED_DDR:
		case IPU_CSI_CLK_MODE_CCIR1120_INTERLACED_SDR:
			csi.csi_mem.interlaced = true;
			break;
		default:
			printk(KERN_ERR "sensor protocol unsupported\n");
			return -EINVAL;
        }

	ipu_csi_get_window_size(cam->ipu, &enc.mem_prp_vf_mem.in_width,
				&enc.mem_prp_vf_mem.in_height, cam->csi);

	/* Configure VDI prp channel */
	enc.mem_prp_vf_mem.out_width = cam->v2f.fmt.pix.width;
	enc.mem_prp_vf_mem.out_height = cam->v2f.fmt.pix.height;

	if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420) {
		enc.mem_prp_vf_mem.out_pixel_fmt = IPU_PIX_FMT_YUV420P;
		pr_info("YUV420\n");
	} else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV422P) {
		enc.mem_prp_vf_mem.out_pixel_fmt = IPU_PIX_FMT_YUV422P;
		pr_info("YUV422P\n");
	} else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {
		enc.mem_prp_vf_mem.out_pixel_fmt = IPU_PIX_FMT_YUYV;
		pr_info("YUYV\n");
	} else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY) {
		enc.mem_prp_vf_mem.out_pixel_fmt = IPU_PIX_FMT_UYVY;
		pr_info("UYVY\n");
	} else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_NV12) {
		enc.mem_prp_vf_mem.out_pixel_fmt = IPU_PIX_FMT_NV12;
		pr_info("NV12\n");
	} else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_BGR24) {
		enc.mem_prp_vf_mem.out_pixel_fmt = IPU_PIX_FMT_BGR24;
		pr_info("BGR24\n");
	} else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24) {
		enc.mem_prp_vf_mem.out_pixel_fmt = IPU_PIX_FMT_RGB24;
		pr_info("RGB24\n");
	} else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB565) {
		enc.mem_prp_vf_mem.out_pixel_fmt = IPU_PIX_FMT_RGB565;
		pr_info("RGB565\n");
	} else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_BGR32) {
		enc.mem_prp_vf_mem.out_pixel_fmt = IPU_PIX_FMT_BGR32;
		pr_info("BGR32\n");
	} else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB32) {
		enc.mem_prp_vf_mem.out_pixel_fmt = IPU_PIX_FMT_RGB32;
		pr_info("RGB32\n");
	} else {
		printk(KERN_ERR "format not supported\n");
		return -EINVAL;
	}

	enc.mem_prp_vf_mem.in_pixel_fmt = enc.mem_prp_vf_mem.out_pixel_fmt;
	enc.mem_prp_vf_mem.motion_sel = MED_MOTION;
	enc.mem_prp_vf_mem.field_fmt = V4L2_FIELD_INTERLACED_BT;

	ipu_csi_enable_mclk_if(cam->ipu, CSI_MCLK_ENC, cam->csi, true, true);

	/* Init CSI channels */
	cam->vdi_enc_first_frame = 1;

	err = ipu_init_channel(cam->ipu, CSI_MEM, &csi);
	if (err != 0) {
		printk(KERN_ERR "CSI_MEM ipu_init_channel %d\n", err);
		return err;
	}

	err = ipu_init_channel_buffer(cam->ipu, CSI_MEM, IPU_OUTPUT_BUFFER,
		enc.mem_prp_vf_mem.out_pixel_fmt, enc.mem_prp_vf_mem.in_width,
		enc.mem_prp_vf_mem.in_height,
		cam->v2f.fmt.pix.bytesperline /
		bytes_per_pixel(enc.mem_prp_vf_mem.out_pixel_fmt),
		IPU_ROTATE_NONE, cam->vdi_enc_bufs[cam->ping_pong_vdi], 0, 0,
		cam->offset.u_offset, cam->offset.v_offset);

	if (err != 0) {
		printk(KERN_ERR "CSI_MEM output buffer\n");
		return err;
	}

	err = ipu_enable_channel(cam->ipu, CSI_MEM);
	if (err < 0) {
		printk(KERN_ERR "ipu_enable_channel CSI_MEM\n");
		return err;
	}

        /* Enable the VDI channels */
	err = ipu_init_channel(cam->ipu, MEM_VDI_PRP_VF_MEM, &enc);
	if (err != 0) {
		printk(KERN_ERR "MEM_VDI_PRP_VF_MEM ipu_init_channel %d\n",
		       err);
		return err;
	}

	err = ipu_init_channel_buffer(cam->ipu, MEM_VDI_PRP_VF_MEM,
			IPU_INPUT_BUFFER,
			enc.mem_prp_vf_mem.in_pixel_fmt,
			enc.mem_prp_vf_mem.in_width,
			enc.mem_prp_vf_mem.in_height,
			cam->v2f.fmt.pix.bytesperline /
			bytes_per_pixel(enc.mem_prp_vf_mem.out_pixel_fmt),
			IPU_ROTATE_NONE,
			cam->vdi_enc_bufs[(cam->ping_pong_vdi + 1) %3], 0, 0,
			cam->offset.u_offset, cam->offset.v_offset);
	if (err != 0) {
		printk(KERN_ERR "MEM_VDI_PRP_VF_MEM output buffer\n");
		return err;
	}
	err = ipu_enable_channel(cam->ipu, MEM_VDI_PRP_VF_MEM);
	if (err < 0) {
		printk(KERN_ERR "ipu_enable_channel MEM_VDI_PRP_VF_MEM\n");
		return err;
	}

	err = ipu_init_channel(cam->ipu, MEM_VDI_PRP_VF_MEM_P, &enc);
	if (err != 0) {
		printk(KERN_ERR "MEM_VDI_PRP_VF_MEM_P ipu_init_channel %d\n",
		       err);
		return err;
	}

	err = ipu_init_channel_buffer(cam->ipu, MEM_VDI_PRP_VF_MEM_P,
			IPU_INPUT_BUFFER,
			enc.mem_prp_vf_mem.in_pixel_fmt,
			enc.mem_prp_vf_mem.in_width,
			enc.mem_prp_vf_mem.in_height,
			cam->v2f.fmt.pix.bytesperline /
			bytes_per_pixel(enc.mem_prp_vf_mem.out_pixel_fmt),
			IPU_ROTATE_NONE,
			cam->vdi_enc_bufs[cam->ping_pong_vdi] +
			cam->v2f.fmt.pix.bytesperline, 0, 0,
			cam->offset.u_offset, cam->offset.v_offset);
	if (err != 0) {
		printk(KERN_ERR "MEM_VDI_PRP_VF_MEM_P input buffer\n");
		return err;
	}

	err = ipu_init_channel_buffer(cam->ipu, MEM_VDI_PRP_VF_MEM_P,
				      IPU_OUTPUT_BUFFER,
				      enc.mem_prp_vf_mem.out_pixel_fmt,
				      enc.mem_prp_vf_mem.out_width,
				      enc.mem_prp_vf_mem.out_height,
				      enc.mem_prp_vf_mem.out_width,
				      IPU_ROTATE_NONE, dummy, dummy, 0,	0, 0);
	if (err != 0) {
		printk(KERN_ERR "MEM_VDI_PRP_VF_MEM_P input buffer\n");
		return err;
	}

	err = ipu_enable_channel(cam->ipu, MEM_VDI_PRP_VF_MEM_P);
	if (err < 0) {
		printk(KERN_ERR "ipu_enable_channel MEM_VDI_PRP_VF_MEM_P\n");
		return err;
	}
	err = ipu_init_channel(cam->ipu, MEM_VDI_PRP_VF_MEM_N, &enc);
	if (err != 0) {
		printk(KERN_ERR "MEM_VDI_PRP_VF_MEM_N ipu_init_channel %d\n",
		       err);
		return err;
	}

	err = ipu_init_channel_buffer(cam->ipu, MEM_VDI_PRP_VF_MEM_N,
			IPU_INPUT_BUFFER,
			enc.mem_prp_vf_mem.in_pixel_fmt,
			enc.mem_prp_vf_mem.in_width,
			enc.mem_prp_vf_mem.in_height,
			cam->v2f.fmt.pix.bytesperline /
			bytes_per_pixel(enc.mem_prp_vf_mem.out_pixel_fmt),
			IPU_ROTATE_NONE,
			cam->vdi_enc_bufs[(cam->ping_pong_vdi + 1) %3] +
			cam->v2f.fmt.pix.bytesperline, 0, 0,
			cam->offset.u_offset, cam->offset.v_offset);
	if (err != 0) {
		printk(KERN_ERR "MEM_VDI_PRP_VF_MEM_N output buffer\n");
		return err;
	}
	err = ipu_enable_channel(cam->ipu, MEM_VDI_PRP_VF_MEM_N);
	if (err < 0) {
		printk(KERN_ERR "ipu_enable_channel MEM_VDI_PRP_VF_MEM_N\n");
		return err;
	}
	return err;
}

/*!
 * function to update physical buffer address for encorder IDMA channel
 *
 * @param eba         physical buffer address for encorder IDMA channel
 * @param buffer_num  int buffer 0 or buffer 1
 *
 * @return  status
 */
static int vdi_enc_eba_update(struct ipu_soc *ipu, dma_addr_t eba,
			      int *buffer_num)
{
	int err = 0;

	pr_debug("eba %x\n", eba);
	err = ipu_update_channel_buffer(ipu, MEM_VDI_PRP_VF_MEM_P,
					IPU_OUTPUT_BUFFER, *buffer_num, eba);
	if (err != 0) {
		ipu_clear_buffer_ready(ipu, MEM_VDI_PRP_VF_MEM_P,
				       IPU_OUTPUT_BUFFER, *buffer_num);
		err = ipu_update_channel_buffer(ipu, MEM_VDI_PRP_VF_MEM_P,
						IPU_OUTPUT_BUFFER,
						*buffer_num, eba);
		if (err != 0) {
			pr_err("ERROR: v4l2 capture: fail to update buf%d\n",
			       *buffer_num);
			return err;
		}
	}

	ipu_select_buffer(ipu, MEM_VDI_PRP_VF_MEM_P, IPU_OUTPUT_BUFFER,
			  *buffer_num);

	*buffer_num = (*buffer_num == 0) ? 1 : 0;
	return 0;
}

/*!
 * Enable encoder task
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  status
 */
static int vdi_enc_enabling_tasks(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0;
	int i;
	CAMERA_TRACE("IPU:In vdi_enc_enabling_tasks\n");

	cam->dummy_frame.vaddress = dma_alloc_coherent(0,
					PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage),
					&cam->dummy_frame.paddress,
					GFP_DMA | GFP_KERNEL);
	if (cam->dummy_frame.vaddress == 0) {
		pr_err("ERROR: v4l2 capture: Allocate dummy frame failed.\n");
		return -ENOBUFS;
	}
	cam->dummy_frame.buffer.type = V4L2_BUF_TYPE_PRIVATE;
	cam->dummy_frame.buffer.length = PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage);
	cam->dummy_frame.buffer.m.offset = cam->dummy_frame.paddress;

	for(i=0;i<3;i++) {
		cam->vdi_enc_bufs_vaddr[i] = dma_alloc_coherent(0,
					PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage),
					&cam->vdi_enc_bufs[i],
					GFP_DMA | GFP_KERNEL);
		if (cam->vdi_enc_bufs_vaddr[i] == 0) {
			pr_err("ERROR: v4l2 capture: Allocate intermediate VDI frame failed.\n");
			return -ENOBUFS;
		}
		cam->vdi_enc_buf_size[i] =
			PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage);
	}
	cam->ping_pong_vdi = 0;

	cam->vdi_wq = create_singlethread_workqueue("vdiq");
	if (!cam->vdi_wq) {
		pr_err( "Could not create VDI work queue\n");
		return -ENOMEM;
	}

	INIT_WORK(&cam->vdi_work, vdi_work_func);

	ipu_clear_irq(cam->ipu, IPU_IRQ_CSI0_OUT_EOF);
	err = ipu_request_irq(cam->ipu, IPU_IRQ_CSI0_OUT_EOF,
			      csi_enc_callback, 0, "Mxc Camera", cam);
	if (err != 0) {
		printk(KERN_ERR "Error registering CSI irq\n");
		return err;
	}

	err = ipu_request_irq(cam->ipu, IPU_IRQ_PRP_VF_OUT_EOF,
			      vdi_enc_callback, 0, "Mxc vdi", cam);
	if (err != 0) {
		printk(KERN_ERR "Error registering VDI irq\n");
		return err;
	}

	err = vdi_enc_setup(cam);
	if (err != 0) {
		printk(KERN_ERR "vdi_enc_setup %d\n", err);
		return err;
	}

	return err;
}

/*!
 * Disable encoder task
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  int
 */
static int vdi_enc_disabling_tasks(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0;
	int i;

	ipu_free_irq(cam->ipu, IPU_IRQ_CSI0_OUT_EOF, cam);
	ipu_free_irq(cam->ipu, IPU_IRQ_PRP_VF_OUT_EOF, cam);

	cancel_work_sync(&cam->vdi_work);
	flush_workqueue(cam->vdi_wq);

	destroy_workqueue(cam->vdi_wq);

	err = ipu_disable_channel(cam->ipu, CSI_MEM, true);
	err = ipu_disable_channel(cam->ipu, MEM_VDI_PRP_VF_MEM_P, true);
	err = ipu_disable_channel(cam->ipu, MEM_VDI_PRP_VF_MEM, true);
	err = ipu_disable_channel(cam->ipu, MEM_VDI_PRP_VF_MEM_N, true);

	ipu_uninit_channel(cam->ipu, CSI_MEM);
	ipu_uninit_channel(cam->ipu, MEM_VDI_PRP_VF_MEM_P);
	ipu_uninit_channel(cam->ipu, MEM_VDI_PRP_VF_MEM);
	ipu_uninit_channel(cam->ipu, MEM_VDI_PRP_VF_MEM_N);

	if (cam->dummy_frame.vaddress != 0) {
		dma_free_coherent(0, cam->dummy_frame.buffer.length,
				  cam->dummy_frame.vaddress,
				  cam->dummy_frame.paddress);
		cam->dummy_frame.vaddress = 0;
	}

	for(i = 0; i < 3; i++) {
		if (cam->vdi_enc_bufs_vaddr[i] != 0) {
			dma_free_coherent(0, cam->vdi_enc_buf_size[i],
					  cam->vdi_enc_bufs_vaddr[i],
					  cam->vdi_enc_bufs[i]);
			cam->vdi_enc_bufs_vaddr[i] = 0;
		}
	}

	ipu_csi_enable_mclk_if(cam->ipu, CSI_MCLK_ENC, cam->csi, false, false);

	return err;
}

/*!
 * Enable csi
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  status
 */
static int vdi_enc_enable_csi(void *private)
{
	cam_data *cam = (cam_data *) private;

	return ipu_enable_csi(cam->ipu, cam->csi);
}

/*!
 * Disable csi
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  status
 */
static int vdi_enc_disable_csi(void *private)
{
	cam_data *cam = (cam_data *) private;

	return ipu_disable_csi(cam->ipu, cam->csi);
}

/*!
 * function to select PRP-ENC as the working path
 *
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  int
 */
int vdi_enc_select(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0;

	if (cam) {
		cam->enc_update_eba = vdi_enc_eba_update;
		cam->enc_enable = vdi_enc_enabling_tasks;
		cam->enc_disable = vdi_enc_disabling_tasks;
		cam->enc_enable_csi = vdi_enc_enable_csi;
		cam->enc_disable_csi = vdi_enc_disable_csi;
	} else {
		err = -EIO;
	}

	return err;
}

/*!
 * function to de-select VDI-ENC as the working path
 *
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  int
 */
int vdi_enc_deselect(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0;

	if (cam) {
		cam->enc_update_eba = NULL;
		cam->enc_enable = NULL;
		cam->enc_disable = NULL;
		cam->enc_enable_csi = NULL;
		cam->enc_disable_csi = NULL;
		if (cam->rot_enc_bufs_vaddr[0]) {
			dma_free_coherent(0, cam->rot_enc_buf_size[0],
					  cam->rot_enc_bufs_vaddr[0],
					  cam->rot_enc_bufs[0]);
			cam->rot_enc_bufs_vaddr[0] = NULL;
			cam->rot_enc_bufs[0] = 0;
		}
		if (cam->rot_enc_bufs_vaddr[1]) {
			dma_free_coherent(0, cam->rot_enc_buf_size[1],
					  cam->rot_enc_bufs_vaddr[1],
					  cam->rot_enc_bufs[1]);
			cam->rot_enc_bufs_vaddr[1] = NULL;
			cam->rot_enc_bufs[1] = 0;
		}
	}

	return err;
}

/*!
 * Init the Encoder channels
 *
 * @return  Error code indicating success or failure
 */
__init int vdi_enc_init(void)
{
	return 0;
}

/*!
 * Deinit the Encoder channels
 *
 */
void __exit vdi_enc_exit(void)
{
}

module_init(vdi_enc_init);
module_exit(vdi_enc_exit);

EXPORT_SYMBOL(vdi_enc_select);
EXPORT_SYMBOL(vdi_enc_deselect);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("IPU VDI ENC Driver");
MODULE_LICENSE("GPL");
