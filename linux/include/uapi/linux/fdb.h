#ifndef __INCLUDE_LINUX_FDB_H__
#define __INCLUDE_LINUX_FDB_H__

/*
 * FDB generic structs userland may want
 */

enum fdb_blit_format {
	FBF_RGB24,
	FBF_RGB32,
	FBF_YUV420_INTERLEAVE,
	FBF_YUV420_SEPARATE, /* YUV420 planar */
	FBF_YUV422_SEPARATE, /* YUV422 planar */
	FBF_YUV444_SEPARATE, /* YUV444 planar */
};

struct fdb_blit {
	uint32_t stride_in;
	uint32_t stride_mask;
	uint32_t stride_out;
	uint32_t width_in; /* 0 = filling, see fill_pixel */
	uint32_t height_in; /* 0 = filling, see fill_pixel */
	uint32_t width_out;
	uint32_t height_out;
	uint32_t bits_per_pixel_in;
	uint32_t bits_per_pixel_out;
	uint32_t fill_pixel; /* only need to set when filling */
	uint32_t rotate;
	uint8_t	 blend; /* blend:0 0->no blend, 1->blend */
	uint32_t src_blend;
	uint32_t dst_blend;
	uint32_t op_blend;
	uint32_t src_x_offset;
	uint32_t src_y_offset;
	uint32_t mask_x_offset;
	uint32_t mask_y_offset;
	uint32_t dest_x_offset;
	uint32_t dest_y_offset;
	uint32_t format_in;
	uint32_t format_out;
	uint64_t src_addr; /* physical ads - note - 64bit when LPAE kernel */
	uint64_t mask_addr; /* physical ads - note - 64bit when LPAE kernel */
	uint64_t dest_addr; /* physical ads - note - 64bit when LPAE kernel */
};

/*
 * This is passed in from userland using fdb blit blocking ioctl
 */

struct blit_block {
	int length; /* userland must set to sizeof(struct blit_block) */
	struct fdb_blit blit;
	uint64_t paddr_in;
	uint32_t gem_handle_in;
	uint32_t offset_in; /* offset inside the gem allocation to use */
	uint32_t gem_handle_mask;
	uint32_t offset_mask; /* offset inside the gem allocation to use */
	uint64_t paddr_out;
	uint32_t gem_handle_out;
	uint32_t offset_out; /* offset inside the gem allocation to use */
};

/*
 * This for blit blend usage
 */
#define GL_ZERO				0x0
#define GL_ONE				0x1
#define GL_SRC_ALPHA			0x302
#define GL_ONE_MINUS_SRC_ALPHA		0x303
#define GL_DST_ALPHA			0x304
#define GL_ONE_MINUS_DST_ALPHA		0x305

#define GL_FUNC_ADD			0x8006
#define GL_FUNC_SUBTRACT		0x800A
#define GL_FUNC_REVERSE_SUBTRACT	0x800B
#define VG_BLEND_DARKEN			0x2007

#endif

