#undef TRACE_SYSTEM
#define TRACE_SYSTEM ipcu

#if !defined(_TRACE_IPCU_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_IPCU_H
#include <linux/tracepoint.h>

/* Send message is also catched by this event */
TRACE_EVENT(ipcu_send_req,

	TP_PROTO(u32 unit, u32 ch, u32 src, u32 *data),

	TP_ARGS(unit, ch, src, data),

	TP_STRUCT__entry(
		__field(	u32,	unit)
		__field(	u32,	ch)
		__field(	u32,	src)
		__field(unsigned long,	data_ptr)
	),

	TP_fast_assign(
		__entry->unit = unit;
		__entry->ch = ch;
		__entry->src = src;
		__entry->data_ptr = (unsigned long)data;
	),

	TP_printk("unit=%u ch=%u src=%u data=0x%lx",
		__entry->unit, __entry->ch, __entry->src, __entry->data_ptr)
);

TRACE_EVENT(ipcu_send_ack,

	TP_PROTO(u32 unit, u32 ch),

	TP_ARGS(unit, ch),

	TP_STRUCT__entry(
		__field(	u32,	unit)
		__field(	u32,	ch)
	),

	TP_fast_assign(
		__entry->unit = unit;
		__entry->ch = ch;
	),

	TP_printk("unit=%u ch=%u", __entry->unit, __entry->ch)
);

TRACE_EVENT(ipcu_handle_rec,

	TP_PROTO(u32 unit, u32 ch, int irq, u32 *data),

	TP_ARGS(unit, ch, irq, data),

	TP_STRUCT__entry(
		__field(	u32,	unit)
		__field(	u32,	ch)
		__field(	int,	irq)
		__field(unsigned long,	data_ptr)
	),

	TP_fast_assign(
		__entry->unit = unit;
		__entry->ch = ch;
		__entry->irq = irq;
		__entry->data_ptr = (unsigned long)data;
	),

	TP_printk("unit=%u ch=%u irq=%d data=%lx",
		 __entry->unit, __entry->ch, __entry->irq, __entry->data_ptr)
);

TRACE_EVENT(ipcu_handle_ack,

	TP_PROTO(u32 unit, u32 ch, int irq),

	TP_ARGS(unit, ch, irq),

	TP_STRUCT__entry(
		__field(	u32,	unit)
		__field(	u32,	ch)
		__field(	int,	irq)
	),

	TP_fast_assign(
		__entry->unit = unit;
		__entry->ch = ch;
		__entry->irq = irq;
	),

	TP_printk("unit=%u ch=%u irq=%d",
		 __entry->unit, __entry->ch, __entry->irq)
);

TRACE_EVENT(ipcu_open_channel,

	TP_PROTO(u32 unit, u32 ch, u32 direction),

	TP_ARGS(unit, ch, direction),

	TP_STRUCT__entry(
		__field(	u32,	unit)
		__field(	u32,	ch)
		__field(	u32,	dir)
	),

	TP_fast_assign(
		__entry->unit = unit;
		__entry->ch = ch;
		__entry->dir = direction
	),

	TP_printk("unit=%u ch=%u direction=%u", __entry->unit, __entry->ch,
		__entry->dir)
);

TRACE_EVENT(ipcu_close_channel,

	TP_PROTO(u32 unit, u32 ch, u32 direction),

	TP_ARGS(unit, ch, direction),

	TP_STRUCT__entry(
		__field(	u32,	unit)
		__field(	u32,	ch)
		__field(	u32,	dir)
	),

	TP_fast_assign(
		__entry->unit = unit;
		__entry->ch = ch;
		__entry->dir = direction
	),

	TP_printk("unit=%u ch=%u direction=%u", __entry->unit, __entry->ch,
		__entry->dir)
);

TRACE_EVENT(ipcu_recv_flush,

	TP_PROTO(u32 unit, u32 ch),

	TP_ARGS(unit, ch),

	TP_STRUCT__entry(
		__field(	u32,	unit)
		__field(	u32,	ch)
	),

	TP_fast_assign(
		__entry->unit = unit;
		__entry->ch = ch;
	),

	TP_printk("unit=%u ch=%u", __entry->unit, __entry->ch)
);

TRACE_EVENT(ipcu_start_recv_msg,

	TP_PROTO(u32 unit, u32 ch, void *buf, u32 flags),

	TP_ARGS(unit, ch, buf, flags),

	TP_STRUCT__entry(
		__field(	u32,	unit)
		__field(	u32,	ch)
		__field(unsigned long,	buf_ptr)
		__field(	u32,	flags)
	),

	TP_fast_assign(
		__entry->unit = unit;
		__entry->ch = ch;
		__entry->buf_ptr = (unsigned long)buf;
		__entry->flags = flags;
	),

	TP_printk("unit=%u ch=%u buf=%lx flags=%x",
		 __entry->unit, __entry->ch, __entry->buf_ptr, __entry->flags)
);

TRACE_EVENT(ipcu_recv_msg,

	TP_PROTO(u32 unit, u32 ch, void *buf, int result),

	TP_ARGS(unit, ch, buf, result),

	TP_STRUCT__entry(
		__field(	u32,	unit)
		__field(	u32,	ch)
		__field(unsigned long,	buf_ptr)
		__field(	int,	result)
	),

	TP_fast_assign(
		__entry->unit = unit;
		__entry->ch = ch;
		__entry->buf_ptr = (unsigned long)buf;
		__entry->result = result;
	),

	TP_printk("unit=%u ch=%u buf=%lx result=%d",
		 __entry->unit, __entry->ch, __entry->buf_ptr, __entry->result)
);

#endif

/* This part must be outside protection */
#include <trace/define_trace.h>
