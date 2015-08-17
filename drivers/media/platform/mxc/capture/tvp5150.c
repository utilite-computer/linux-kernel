/*
 * tvp5150 - Texas Instruments TVP5150A/AM1 video decoder driver
 *
 * Copyright (c) 2005,2006 Mauro Carvalho Chehab (mchehab@infradead.org)
 * This code is placed under the terms of the GNU General Public License v2
 */

#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <media/v4l2-device.h>
#include <media/tvp5150.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-ctrls.h>

#include <media/v4l2-int-device.h>
#include <linux/fsl_devices.h>

#include "tvp5150_reg.h"

MODULE_DESCRIPTION("Texas Instruments TVP5150A video decoder driver");
MODULE_AUTHOR("CompuLab");
MODULE_LICENSE("GPL");

#define TVP5150_XCLK_MIN 6000000
#define TVP5150_XCLK_MAX 27000000

#define TVP5150_WINDOW_WIDTH_DEF	720
#define TVP5150_WINDOW_HEIGHT_DEF	525
#define TVP5150_DEFAULT_FPS	30

#define NTSC_NUM_ACTIVE_PIXELS  (720)
#define NTSC_NUM_ACTIVE_LINES   (525)
#define PAL_NUM_ACTIVE_PIXELS   (720)
#define PAL_NUM_ACTIVE_LINES    (576)

static int debug;
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

/* enum tvp515x_std - enum for supported standards */
enum tvp515x_std {
	STD_PAL_BDGHIN = 0,
	STD_NTSC_MJ,
	STD_INVALID
};

/*!
 * struct tvp515x_std_info - Structure to store standard informations
 * @width: Line width in pixels
 * @height:Number of active lines
 * @video_std: Value to write in REG_VIDEO_STD register
 * @standard: v4l2 standard structure information
 */
struct tvp515x_std_info {
	u8 video_std;
	struct v4l2_standard standard;
	struct v4l2_pix_format pix;
	/* struct v4l2_mbus_framefmt format;*/
};

/*!
 * Supported standards -
 *
 * Currently supports two standards only, need to add support for rest of the
 * modes, like SECAM, etc...
 */
static struct tvp515x_std_info tvp515x_std_list[] = {
	/* Standard: STD_NTSC_MJ */
	/* Standard: STD_PAL_BDGHIN */
	[STD_PAL_BDGHIN] = {
		.video_std = VIDEO_STD_PAL_BDGHIN_BIT,
		.standard = {
			.index = 1,
			.id = V4L2_STD_PAL,
			.name = "PAL",
			.frameperiod = {1, 25},
			.framelines = 625
		},
		.pix = {
			.width = PAL_NUM_ACTIVE_PIXELS,
			.height = PAL_NUM_ACTIVE_LINES,
			.pixelformat = V4L2_PIX_FMT_UYVY,
			.field = V4L2_FIELD_INTERLACED,
			.colorspace = V4L2_COLORSPACE_SMPTE170M,
		},
	},
	[STD_NTSC_MJ] = {
		.video_std = VIDEO_STD_NTSC_MJ_BIT,
		.standard = {
			.index = 0,
			.id = V4L2_STD_NTSC,
			.name = "NTSC",
			.frameperiod = {1001, 30000},
			.framelines = 525
		},
		.pix = {
			.width = NTSC_NUM_ACTIVE_PIXELS,
			.height = NTSC_NUM_ACTIVE_LINES,
			.pixelformat = V4L2_PIX_FMT_UYVY,
			.field = V4L2_FIELD_INTERLACED,
			.colorspace = V4L2_COLORSPACE_SMPTE170M,
		},
	},
	/* Standard: need to add for additional standard */
};

/*
 * Maintains the information on the current state of the sensor.
 */
static struct sensor {
	const void *platform_data;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	struct v4l2_captureparm streamcap;
	bool on;

	/* control settings */
	int brightness;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;

	u32 mclk;
	u8 mclk_source;
	struct clk *sensor_clk;
	int csi;

	/* Current set standard */
	v4l2_std_id norm;
	u32 input;
	u32 output;
	int enable;
} tvp5150_data;

static int tvp5150_read_werr(struct i2c_client *c, unsigned char addr)
{
	unsigned char buffer[1];
	int err;

	buffer[0] = addr;
	err = i2c_master_send(c, buffer, 1);
	if (err != 1)
		return -EIO;

	msleep(10);

	err = i2c_master_recv(c, buffer, 1);
	if (err != 1)
		return -EIO;

	return buffer[0];
}

static inline int tvp5150_write_werr(struct i2c_client *c, unsigned char addr,
				 unsigned char value)
{
	unsigned char buffer[2];
	int rc;

	buffer[0] = addr;
	buffer[1] = value;
	v4l_dbg(2, debug, c, "tvp5150: writing 0x%02x 0x%02x\n",
		buffer[0], buffer[1]);
	if (2 != (rc = i2c_master_send(c, buffer, 2)))
		v4l_dbg(0, debug, c,
			"i2c i/o error: rc == %d (should be 2)\n", rc);

	return rc;
}

/****************************************************************************
			Basic functions
 ****************************************************************************/

struct i2c_reg_value {
	unsigned char reg;
	unsigned char value;
};

/* Default values as sugested at TVP5150AM1 datasheet */
static const struct i2c_reg_value tvp5150_init_default[] = {
	{ /* 0x00 */
		TVP5150_VD_IN_SRC_SEL_1,0x00
	},
	{ /* 0x01 */
		TVP5150_ANAL_CHL_CTL,0x15
	},
	{ /* 0x02 */
		TVP5150_OP_MODE_CTL,0x00
	},
	{ /* 0x03 */
		TVP5150_MISC_CTL,0x01
	},
	{ /* 0x06 */
		TVP5150_COLOR_KIL_THSH_CTL,0x10
	},
	{ /* 0x07 */
		TVP5150_LUMA_PROC_CTL_1,0x60
	},
	{ /* 0x08 */
		TVP5150_LUMA_PROC_CTL_2,0x00
	},
	{ /* 0x09 */
		TVP5150_BRIGHT_CTL,0x80
	},
	{ /* 0x0a */
		TVP5150_SATURATION_CTL,0x80
	},
	{ /* 0x0b */
		TVP5150_HUE_CTL,0x00
	},
	{ /* 0x0c */
		TVP5150_CONTRAST_CTL,0x80
	},
	{ /* 0x0d */
		TVP5150_DATA_RATE_SEL,0x47
	},
	{ /* 0x0e */
		TVP5150_LUMA_PROC_CTL_3,0x00
	},
	{ /* 0x0f */
		TVP5150_CONF_SHARED_PIN,0x08
	},
	{ /* 0x11 */
		TVP5150_ACT_VD_CROP_ST_MSB,0x00
	},
	{ /* 0x12 */
		TVP5150_ACT_VD_CROP_ST_LSB,0x00
	},
	{ /* 0x13 */
		TVP5150_ACT_VD_CROP_STP_MSB,0x00
	},
	{ /* 0x14 */
		TVP5150_ACT_VD_CROP_STP_LSB,0x00
	},
	{ /* 0x15 */
		TVP5150_GENLOCK,0x01
	},
	{ /* 0x16 */
		TVP5150_HORIZ_SYNC_START,0x80
	},
	{ /* 0x18 */
		TVP5150_VERT_BLANKING_START,0x00
	},
	{ /* 0x19 */
		TVP5150_VERT_BLANKING_STOP,0x00
	},
	{ /* 0x1a */
		TVP5150_CHROMA_PROC_CTL_1,0x0c
	},
	{ /* 0x1b */
		TVP5150_CHROMA_PROC_CTL_2,0x14
	},
	{ /* 0x1c */
		TVP5150_INT_RESET_REG_B,0x00
	},
	{ /* 0x1d */
		TVP5150_INT_ENABLE_REG_B,0x00
	},
	{ /* 0x1e */
		TVP5150_INTT_CONFIG_REG_B,0x00
	},
	{ /* 0x28 */
		TVP5150_VIDEO_STD,0x00
	},
	{ /* 0x2e */
		TVP5150_MACROVISION_ON_CTR,0x0f
	},
	{ /* 0x2f */
		TVP5150_MACROVISION_OFF_CTR,0x01
	},
	{ /* 0xbb */
		TVP5150_TELETEXT_FIL_ENA,0x00
	},
	{ /* 0xc0 */
		TVP5150_INT_STATUS_REG_A,0x00
	},
	{ /* 0xc1 */
		TVP5150_INT_ENABLE_REG_A,0x00
	},
	{ /* 0xc2 */
		TVP5150_INT_CONF,0x04
	},
	{ /* 0xc8 */
		TVP5150_FIFO_INT_THRESHOLD,0x80
	},
	{ /* 0xc9 */
		TVP5150_FIFO_RESET,0x00
	},
	{ /* 0xca */
		TVP5150_LINE_NUMBER_INT,0x00
	},
	{ /* 0xcb */
		TVP5150_PIX_ALIGN_REG_LOW,0x4e
	},
	{ /* 0xcc */
		TVP5150_PIX_ALIGN_REG_HIGH,0x00
	},
	{ /* 0xcd */
		TVP5150_FIFO_OUT_CTRL,0x01
	},
	{ /* 0xcf */
		TVP5150_FULL_FIELD_ENA,0x00
	},
	{ /* 0xd0 */
		TVP5150_LINE_MODE_INI,0x00
	},
	{ /* 0xfc */
		TVP5150_FULL_FIELD_MODE_REG,0x7f
	},
	{ /* end of data */
		0xff,0xff
	}
};

/* Default values as sugested at TVP5150AM1 datasheet */
static const struct i2c_reg_value tvp5150_init_enable[] = {
	{
		TVP5150_CONF_SHARED_PIN, 2
	},{	/* Automatic offset and AGC enabled */
		TVP5150_ANAL_CHL_CTL, 0x15
	},{	/* Activate YCrCb output 0x9 or 0xd ? */
		TVP5150_MISC_CTL, 0x6f
	},{	/* Activates video std autodetection for all standards */
		TVP5150_AUTOSW_MSK, 0x0
	},{	/* Default format: 0x47. For 4:2:2: 0x40 */
		TVP5150_DATA_RATE_SEL, 0x47
	},{
		TVP5150_CHROMA_PROC_CTL_1, 0x0c
	},{
		TVP5150_CHROMA_PROC_CTL_2, 0x54
	},{	/* Non documented, but initialized on WinTV USB2 */
		0x27, 0x20
	},{
		0xff,0xff
	}
};

struct tvp5150_vbi_type {
	unsigned int vbi_type;
	unsigned int ini_line;
	unsigned int end_line;
	unsigned int by_field:1;
};

struct i2c_vbi_ram_value {
	u16 reg;
	struct tvp5150_vbi_type type;
	unsigned char values[16];
};

/*
 * This struct have the values for each supported VBI Standard
 * by tvp5150_vbi_types should follow the same order as vbi_ram_default
 * value 0 means rom position 0x10, value 1 means rom position 0x30
 * and so on. There are 16 possible locations from 0 to 15.
 */

static struct i2c_vbi_ram_value vbi_ram_default[] =
{
	{0x030, /* Teletext, PAL, WST System B */
		{V4L2_SLICED_TELETEXT_B,6,22,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0x27, 0x2e, 0x20, 0x2b,
		  0xa6, 0x72, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
	{0x0f0, /* Closed Caption, NTSC */
		{V4L2_SLICED_CAPTION_525,21,21,1},
		{ 0xaa, 0x2a, 0xff, 0x3f, 0x04, 0x51, 0x6e, 0x02,
		  0x69, 0x8c, 0x09, 0x00, 0x00, 0x00, 0x27, 0x00 }
	},
	{0x110, /* Wide Screen Signal, PAL/SECAM */
		{V4L2_SLICED_WSS_625,23,23,1},
		{ 0x5b, 0x55, 0xc5, 0xff, 0x00, 0x71, 0x6e, 0x42,
		  0xa6, 0xcd, 0x0f, 0x00, 0x00, 0x00, 0x3a, 0x00 }
	},
	{0x190, /* Video Program System (VPS), PAL */
		{V4L2_SLICED_VPS,16,16,0},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xba, 0xce, 0x2b, 0x0d,
		  0xa6, 0xda, 0x0b, 0x00, 0x00, 0x00, 0x60, 0x00 }
	},
	/* 0x1d0 User programmable */

	/* End of struct */
	{ (u16)-1 }
};

/****************************************************************************
			I2C Command
 ****************************************************************************/
#define v4l2_get_client(S) ((struct sensor *)(S)->priv)->i2c_client

static inline void tvp5150_write(struct v4l2_int_device *s, unsigned char addr,
				unsigned char value)
{
	struct i2c_client *c = v4l2_get_client(s);
	int ret;

	ret = tvp5150_write_werr(c, addr, value);
	if (ret == -EIO)
		v4l_err(c, "i2c i/o write error\n");

	return;
}

static int tvp5150_read(struct v4l2_int_device *s, unsigned char addr)
{
	struct i2c_client *c = v4l2_get_client(s);
	int ret;

	ret = tvp5150_read_werr(c, addr);
	if (ret == -EIO) {
		v4l_err(c, "i2c i/o read error\n");
		return -1;
	}

	v4l_dbg(2, debug, c, "tvp5150: read 0x%02x = 0x%02x\n", addr, ret);

	return ret;
}

static int tvp5150_write_inittab(struct v4l2_int_device *s,
				const struct i2c_reg_value *regs)
{
	while (regs->reg != 0xff) {
		tvp5150_write(s, regs->reg, regs->value);
		regs++;
	}
	return 0;
}

static int tvp5150_vdp_init(struct v4l2_int_device *s,
				const struct i2c_vbi_ram_value *regs)
{
	unsigned int i;

	/* Disable Full Field */
	tvp5150_write(s, TVP5150_FULL_FIELD_ENA, 0);

	/* Before programming, Line mode should be at 0xff */
	for (i = TVP5150_LINE_MODE_INI; i <= TVP5150_LINE_MODE_END; i++)
		tvp5150_write(s, i, 0xff);

	/* Load Ram Table */
	while (regs->reg != (u16)-1) {
		tvp5150_write(s, TVP5150_CONF_RAM_ADDR_HIGH, regs->reg >> 8);
		tvp5150_write(s, TVP5150_CONF_RAM_ADDR_LOW, regs->reg);

		for (i = 0; i < 16; i++)
			tvp5150_write(s, TVP5150_VDP_CONF_RAM_DATA,
				      regs->values[i]);

		regs++;
	}
	return 0;
}

static inline void tvp5150_selmux(struct v4l2_int_device *s)
{
	int opmode = 0;
	int input = 0;
	unsigned char val;

	if ((tvp5150_data.output & TVP5150_BLACK_SCREEN) ||
	    !tvp5150_data.enable)
		input = 8;

	switch (tvp5150_data.input) {
	case TVP5150_COMPOSITE1:
		input |= 2;
		/* fall through */
	case TVP5150_COMPOSITE0:
		break;
	case TVP5150_SVIDEO:
	default:
		input |= 1;
		break;
	}

	printk("Selecting video route: route input=%i, output=%i "
			"=> tvp5150 input=%i, opmode=%i\n",
			tvp5150_data.input,
			tvp5150_data.output,
			input, opmode);

	tvp5150_write(s, TVP5150_OP_MODE_CTL, opmode);
	tvp5150_write(s, TVP5150_VD_IN_SRC_SEL_1, input);

	/*
	 * Svideo should enable YCrCb output and disable GPCL output
	 * For Composite and TV, it should be the reverse
	 */
	val = tvp5150_read(s, TVP5150_MISC_CTL);
	if (tvp5150_data.input == TVP5150_SVIDEO)
		val = (val & ~0x40) | 0x10;
	else
		val = (val & ~0x10) | 0x40;

	tvp5150_write(s, TVP5150_MISC_CTL, val);
};

static int tvp5150_set_std(struct v4l2_int_device *s, v4l2_std_id std)
{
	int fmt = 0;

	tvp5150_data.norm = std;

	/* First tests should be against specific std */

	if (std == V4L2_STD_ALL) {
		fmt = 0;	/* Autodetect mode */
	} else if (std & V4L2_STD_NTSC_443) {
		fmt = 0xa;
	} else if (std & V4L2_STD_PAL_M) {
		fmt = 0x6;
	} else if (std & (V4L2_STD_PAL_N | V4L2_STD_PAL_Nc)) {
		fmt = 0x8;
	} else {
		/* Then, test against generic ones */
		if (std & V4L2_STD_NTSC)
			fmt = 0x2;
		else if (std & V4L2_STD_PAL)
			fmt = 0x4;
		else if (std & V4L2_STD_SECAM)
			fmt = 0xc;
	}

	printk("Set video std register to %d.\n", fmt);
	tvp5150_write(s, TVP5150_VIDEO_STD, fmt);
	return 0;
}

static int tvp5150_reset_init(struct v4l2_int_device *s, u32 val)
{
	/* Initializes TVP5150 to its default values */
	tvp5150_write_inittab(s, tvp5150_init_default);

	/* Initializes VDP registers */
	tvp5150_vdp_init(s, vbi_ram_default);

	/* Selects decoder input */
	tvp5150_selmux(s);

	/* Initializes TVP5150 to stream enabled values */
	tvp5150_write_inittab(s, tvp5150_init_enable);

	/* Initialize image preferences */
	tvp5150_set_std(s, tvp5150_data.norm);
	return 0;
};

/**
 * tvp515x_query_current_std() : Query the current standard detected by TVP5151
 * @sd: ptr to v4l2_subdev struct
 *
 * Returns the current standard detected by TVP5151, STD_INVALID if there is no
 * standard detected.
 */
static int tvp515x_query_current_std(struct v4l2_int_device *s)
{
	u8 std, std_status;

	std = tvp5150_read(s, TVP5150_VIDEO_STD);
	if ((std & VIDEO_STD_MASK) == VIDEO_STD_AUTO_SWITCH_BIT)
		/* use the standard status register */
		std_status = tvp5150_read(s, TVP5150_STATUS_REG_5);
	else
		/* use the standard register itself */
		std_status = std;

	switch (std_status & VIDEO_STD_MASK) {
	case VIDEO_STD_NTSC_MJ_BIT:
	case VIDEO_STD_NTSC_MJ_BIT_AS:
		return STD_NTSC_MJ;

	case VIDEO_STD_PAL_BDGHIN_BIT:
	case VIDEO_STD_PAL_BDGHIN_BIT_AS:
		return STD_PAL_BDGHIN;

	default:
		return STD_INVALID;
	}

	return STD_INVALID;
}

static int ioctl_dev_init(struct v4l2_int_device *s)
{
	tvp5150_reset_init(s, 0);
	return 0;
}

static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	/* Initializes TVP5150 to its default values */
	/* set PCLK (27MHz) */
	tvp5150_write(s, TVP5150_CONF_SHARED_PIN, 0x00);

	/* Output format: 8-bit ITU-R BT.656 with embedded syncs */
	if (on)
		tvp5150_write(s, TVP5150_MISC_CTL, 0x09);
	else
		tvp5150_write(s, TVP5150_MISC_CTL, 0x00);

	return 0;
}

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = tvp5150_data.mclk;
	pr_debug("   clock_curr=mclk=%d\n", tvp5150_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = TVP5150_XCLK_MIN;
	p->u.bt656.clock_max = TVP5150_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	return 0;
}

/*
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	tvp5150_reset_init(s, 0);
	return 0;
}

/*
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	if (fmt->index > 0)	/* only 1 pixelformat support so far */
		return -EINVAL;

	fmt->pixelformat = tvp5150_data.pix.pixelformat;

	return 0;
}

/*
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor *sensor = s->priv;
	/* query the current standard */
	int current_std = tvp515x_query_current_std(s);
	if (current_std != STD_INVALID)
		sensor->pix = tvp515x_std_list[current_std].pix;

	/* Update the defaults */
	tvp5150_data.pix = sensor->pix;
	f->fmt.pix = tvp5150_data.pix;
	return 0;
}

/*
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	int ret = 0;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	/* s->priv points to platform_data */

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("	type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");

		/*
		 * Check that the new frame rate is allowed.
		 * Changing the frame rate is not allowed on this
		 * camera.
		 */
		if (cparm->timeperframe.denominator !=
		    tvp5150_data.streamcap.timeperframe.denominator) {
			pr_err("ERROR: tvp5150: ioctl_s_parm: " \
			       "This camera does not allow frame rate "
			       "changes.\n");
			ret = -EINVAL;
		} else {
			tvp5150_data.streamcap.timeperframe =
						cparm->timeperframe;
		      /* Call any camera functions to match settings. */
		}

		/* Check that new capture mode is supported. */
		if ((cparm->capturemode != 0) &&
		    !(cparm->capturemode & V4L2_MODE_HIGHQUALITY)) {
			pr_err("ERROR: tvp5150: ioctl_s_parm: " \
				"unsupported capture mode\n");
			ret  = -EINVAL;
		} else {
			tvp5150_data.streamcap.capturemode =
						cparm->capturemode;
		      /* Call any camera functions to match settings. */
		      /* Right now this camera only supports 1 mode. */
		}
		printk("%s TVP5150_OUTPUT_CONTROL_CHIP_ENABLE\n",__func__);
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_err("   type is not V4L2_BUF_TYPE_VIDEO_CAPTURE " \
			"but %d\n", a->type);
		ret = -EINVAL;
		break;

	default:
		pr_err("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return 0;
}

/*
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = tvp5150_data.brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = tvp5150_data.hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = tvp5150_data.contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = tvp5150_data.saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = tvp5150_data.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = tvp5150_data.blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = tvp5150_data.ae_mode;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		tvp5150_write(s, TVP5150_BRIGHT_CTL, vc->value);
		return 0;
	case V4L2_CID_CONTRAST:
		tvp5150_write(s, TVP5150_CONTRAST_CTL, vc->value);
		return 0;
	case V4L2_CID_SATURATION:
		tvp5150_write(s, TVP5150_SATURATION_CTL, vc->value);
		return 0;
	case V4L2_CID_HUE:
		tvp5150_write(s, TVP5150_HUE_CTL, vc->value);
		return 0;
	}
	return -EINVAL;
}

/*
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index > 0)
		return -EINVAL;

	fsize->pixel_format = tvp5150_data.pix.pixelformat;
	fsize->discrete.width = tvp5150_data.pix.width;
	fsize->discrete.height = tvp5150_data.pix.height;
	fsize->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	return 0;
}

/*
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
	if (fival->index != 0) {
		return -EINVAL;
	}

	if (fival->pixel_format == 0 ||
	    fival->width == 0 || fival->height == 0) {
		pr_warning("Please assign pixelformat, width and height.\n");
		return -EINVAL;
	}

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;

	if (fival->pixel_format == tvp5150_data.pix.pixelformat) {
		fival->discrete.denominator =
			tvp5150_data.streamcap.timeperframe.denominator;
		return 0;
	}
	return -EINVAL;
}

/*
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
	       "tvp5150_camera");

	return 0;
}

/*
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc tvp5150_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *)ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func *)ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *)ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
			(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func *)ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
			(v4l2_int_ioctl_func *)ioctl_enum_fmt_cap},
/*	{vidioc_int_try_fmt_cap_num,
			(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_g_fmt_cap},
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_s_fmt_cap}, */
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *)ioctl_s_parm},
/*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, */
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *)ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
			(v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
			(v4l2_int_ioctl_func *)ioctl_enum_frameintervals},
	{vidioc_int_g_chip_ident_num,
			(v4l2_int_ioctl_func *)ioctl_g_chip_ident},
};

static struct v4l2_int_slave tvp5150_slave = {
	.ioctls = tvp5150_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(tvp5150_ioctl_desc),
};

static struct v4l2_int_device tvp5150_int_device = {
	.module = THIS_MODULE,
	.name = "tvp5150",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &tvp5150_slave,
	},
};

static int tvp5150_probe(struct i2c_client *c,
			 const struct i2c_device_id *id)
{
	int retval;
	struct device *dev = &c->dev;
	int msb_id, lsb_id, msb_rom, lsb_rom;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(c->adapter,
	     I2C_FUNC_SMBUS_READ_BYTE | I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -EIO;

	msb_id = tvp5150_read_werr(c, TVP5150_MSB_DEV_ID);
	if (msb_id == -EIO)
		return -ENODEV;

	lsb_id = tvp5150_read_werr(c, TVP5150_LSB_DEV_ID);
	if (lsb_id == -EIO)
		return -ENODEV;

	v4l_info(c, "chip found at address 0x%02x (%s)\n",
		 c->addr, c->adapter->name);

	msb_rom = tvp5150_read_werr(c, TVP5150_ROM_MAJOR_VER);
	lsb_rom = tvp5150_read_werr(c, TVP5150_ROM_MINOR_VER);

	if (msb_rom == 4 && lsb_rom == 0) { /* Is TVP5150AM1 */
		v4l_info(c, "tvp%02x%02xam1 detected.\n", msb_id, lsb_id);
		/* ITU-T BT.656.4 timing */
		tvp5150_write_werr(c, TVP5150_REV_SELECT, 0);
	} else {
		if (msb_rom == 3 || lsb_rom == 0x21) { /* Is TVP5150A */
			v4l_info(c, "tvp%02x%02xa detected.\n", msb_id, lsb_id);
		} else {
			v4l_info(c, "*** tvp%02x%02x chip detected.\n",
					msb_id, lsb_id);
			v4l_info(c, "*** Rom ver is %d.%d\n", msb_rom, lsb_rom);
		}
	}

	/* Set initial values for the sensor struct. */
	memset(&tvp5150_data, 0, sizeof(tvp5150_data));
	tvp5150_data.mclk = TVP5150_XCLK_MAX;
	tvp5150_data.mclk = 0;
	tvp5150_data.mclk_source = 0;

	tvp5150_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(tvp5150_data.sensor_clk)) {
		dev_err(dev, "get mclk failed\n");
		return PTR_ERR(tvp5150_data.sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(tvp5150_data.csi));
	if (retval) {
		dev_err(dev, "csi_id invalid\n");
		return retval;
	}

	tvp5150_data.norm = V4L2_STD_ALL;	/* Default is autodetect */
	tvp5150_data.input = TVP5150_COMPOSITE0;
	tvp5150_data.enable = 1;

	tvp5150_data.i2c_client = c;
	tvp5150_data.pix.pixelformat = V4L2_PIX_FMT_UYVY;
	tvp5150_data.pix.colorspace = V4L2_COLORSPACE_SMPTE170M;
	tvp5150_data.pix.field = V4L2_FIELD_NONE;
	tvp5150_data.pix.width = TVP5150_WINDOW_WIDTH_DEF;
	tvp5150_data.pix.height = TVP5150_WINDOW_HEIGHT_DEF;

	tvp5150_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					V4L2_CAP_TIMEPERFRAME;

	tvp5150_data.streamcap.capturemode = 0;
	tvp5150_data.streamcap.timeperframe.denominator = TVP5150_DEFAULT_FPS;
	tvp5150_data.streamcap.timeperframe.numerator = 1;

	clk_prepare_enable(tvp5150_data.sensor_clk);

	tvp5150_int_device.priv = &tvp5150_data;
	retval = v4l2_int_device_register(&tvp5150_int_device);
	pr_debug("camera tvp5150 is found v4l2 res %d\n",retval);

	clk_disable_unprepare(tvp5150_data.sensor_clk);
	return retval;
}

static int tvp5150_remove(struct i2c_client *c)
{
	v4l_dbg(1, debug, c,
		"tvp5150.c: removing tvp5150 adapter on address 0x%x\n",
		c->addr << 1);

	v4l2_int_device_unregister(&tvp5150_int_device);

	return 0;
}

/* ----------------------------------------------------------------------- */

static const struct i2c_device_id tvp5150_id[] = {
	{ "tvp5150", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tvp5150_id);

static struct i2c_driver tvp5150_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tvp5150",
	},
	.probe		= tvp5150_probe,
	.remove		= tvp5150_remove,
	.id_table	= tvp5150_id,
};

module_i2c_driver(tvp5150_driver);
