#ifndef __MXC_EPDC_HELPER_H__
#define __MXC_EPDC_HELPER_H__
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/completion.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/fs.h>

#include <uapi/linux/mxc_epdc_helper.h>

struct emr_point;

void mxc_epdc_powerup(void);
void mxc_epdc_powerdown(void);
int mxc_epdc_enable_fast_mode(void);
int mxc_epdc_disable_fast_mode(void);
bool mxc_epdc_test_underrun(void);
bool mxc_epdc_test_collision(void);
bool mxc_epdc_busy(void);
bool mxc_epdc_have_free_lut(void);
void mxc_epdc_free_luts(void);
int mxc_epdc_update_fast(int x, int y, int xh, int yw, int mode);
bool mxc_epdc_test_normal_mode(void);
bool mxc_epdc_test_fast_mode(void);
bool mxc_epdc_test_app_mode(void);
int mxc_epdc_enable_app_mode(void);
int mxc_epdc_disable_app_mode(void);
int mxc_epdc_upd_buffer_num(void);
int mxc_epdc_max_upd_buffer_num(void);
int mxc_epdc_jump_to_normal(void);
void mxc_epdc_lock_display_flag(void);
void mxc_epdc_unlock_display_flag(void);
void mxc_epdc_set_fast_mode(void);
void mxc_epdc_clear_fast_mode(void);
void mxc_epdc_clear_normal_mode(void);
void mxc_epdc_clear_app_mode(void);
bool mxc_epdc_cur_update(void);
int mxc_epdc_free_list_count(void);
int mxc_epdc_pending_list_count(void);
int mxc_epdc_collision_count(void);

int epdc_docker_jump_to_fast(void);
void epdc_helper_feed_usr(struct emr_point *pp, bool force);
#ifdef CONFIG_FB_MXC_EPDC_HELPER
extern int mxc_epdc_helper_probe(void *addr, int xvirtual, int yvirtual,
				 int max_width, int max_height);
extern int mxc_epdc_helper_remove(void);
void epdc_helper_add_point(int x, int y, int p, int tool, u8 pen, int f1, int f2);
void epdc_helper_setup_emr(int width, int height);
#else
static inline int mxc_epdc_helper_probe(void *addr, int xvirtual, int yvirtual,
					int max_width, int max_height)
{
	return 0;
}

static inline int mxc_epdc_helper_remove(void)
{
	return 0;
}

static inline void epdc_helper_add_point(int x, int y,
					 int p, int tool,
					 u8 pen, int f1, int f2) {}
static inline void epdc_helper_setup_emr(int width, int height){}
#endif

struct emr_point {
	int x, y, raw_x, raw_y, p, tool, f1, f2;
	u8 pen, color;
	struct list_head list;
};

struct epdc_cache {
	/* size of data unit */
	int size;
	/* count of data unit */
	int count;
	/* used count of data unit */
	int used;
	/* start zero bit */
	int start;
	void *cache;
	volatile unsigned long *map;
	spinlock_t lock;
};

struct emr_helper {
	struct emr_point	begin_point;
	struct emr_point	end_point;
	struct emr_point	last_point;
	int			max_width;
	int			max_height;
	bool			pen_down;
	/* count for filter point pressure */
	int			filter_count;
	/* pre pressure for filter point pressure */
	int			filter_pressure;
	/* rubber or pen */
	int			tool, f1, f2;
	/* jiffies for tool shaking */
	unsigned long		tool_jiffs;
	/* emr point number */
	int			emr_number;
	int			delta_x, delta_y;
	/* emr point map and size */
	unsigned long		*map;
	int			map_size;
	struct file		*emr_fp;
	struct file		*read_fp;
	unsigned int		emr_cnt;
	unsigned int		read_cnt;
	unsigned int		draw_cnts;
	unsigned int		feed_cnts;
	unsigned int		eat_cnts;
	unsigned int		eat_fails;
	unsigned int		return_cnts;
	unsigned int		q_flush_cnts;
	unsigned int		pid, pid2, pid3;
};

struct mxc_epdc_step {
	int epdc_helper_func;
	int epdc_helper_eat_point;
	int epdc_docker_update;
	int epdc_docker_end_direct;
};

struct mxc_epdc_helper {
	dev_t			devt;
	struct device		*dev;
	spinlock_t		emr_lock;
	spinlock_t		usr_lock;
	int			users;
	bool			active;
	bool			debug;
	bool			record;
	bool			suspended;

	struct task_struct	*helper;
	struct semaphore	helper_sema;
	struct completion	helper_done;
	void			*priv;
	struct emr_helper	emr_helper;
	struct list_head	emr_head;
	struct list_head	usr_head;

	wait_queue_head_t	wait;
	struct epdc_cache	*emr_cache;

	int			pen_type, pen_type_copy;
	int			pen_width, pen_width_copy;
	int			pen_color, pen_color_copy;
	/* data struct ready */
	bool			ready;
	int			max_width;
	int			max_height;
	/* fast lookup */
	int			*row_starts;
	int			*virt_row_starts;
	/* invalid region map and size */
	unsigned long		*map;
	int			map_size;

	unsigned int		stop_cnts;

	unsigned int		display_mutex_refs;
	unsigned int		docker_mutex_refs;
	unsigned int		pxp_delay;

	struct write_disables	disables;
	struct write_disables	tmp_disables;
	struct delayed_work	watchdog;
	struct mxc_epdc_step	step;
};
extern struct mxc_epdc_helper *epdc_helper;

static inline void epdc_helper_do_record(struct file *filp,
					 struct emr_point *pp)
{
	loff_t pos;
	char buf[96];

	if (filp) {
		memset(buf, 0, sizeof(buf));
		snprintf(buf, sizeof(buf) - 1,
			 "%d,%d,%d\n", pp->x, pp->y, pp->p);
		pos = filp->f_pos;
		__kernel_write(filp,
			       buf, strlen(buf), &pos);
		filp->f_pos = pos;
	}
}

static inline void epdc_helper_record_emr(struct emr_point *pp)
{
	struct emr_helper *helper = &epdc_helper->emr_helper;

	epdc_helper_do_record(helper->emr_fp, pp);
}

static inline __maybe_unused bool epdc_test_writeable(int x, int y)
{
	if ((x < 0) ||
	    (x >= epdc_helper->max_width) ||
	    (y < 0) ||
	    (y >= epdc_helper->max_height))
		return false;

	return !test_bit(epdc_helper->row_starts[y] + x, epdc_helper->map);
}

static inline __maybe_unused int epdc_cache_remain(struct epdc_cache *cache)
{
	return cache->count - cache->used;
}

static inline __maybe_unused void *epdc_cache_alloc(struct epdc_cache *cache)
{
	int id;

	spin_lock(&cache->lock);
	id = find_next_zero_bit((const void *)(cache->map),
				cache->count, cache->start);
	if (id >= 0 && id < cache->count) {
		set_bit(id, cache->map);
		cache->start = ((id + 1) < cache->count) ?
			(id + 1) : 0;
		cache->used++;
		spin_unlock(&cache->lock);
		return &((char *)(cache->cache))[id * cache->size];
	}
	spin_unlock(&cache->lock);

	return NULL;
}

static inline __maybe_unused void epdc_cache_free(struct epdc_cache *cache,
						  void *addr)
{
	int id;

	id = ((char *)addr - (char *)(cache->cache)) / cache->size;
	if (id >= 0 && id < cache->count) {
		spin_lock(&cache->lock);
		clear_bit(id, cache->map);
		if (id < cache->start)
			cache->start = id;
		cache->used--;
		spin_unlock(&cache->lock);
		return;
	}
}

static __maybe_unused struct epdc_cache *epdc_cache_create(int size, int count)
{
	struct epdc_cache *cache;

	if (size <= 0 || count <= 0)
		return NULL;

	cache = kzalloc(sizeof(*cache), GFP_KERNEL);
	if (!cache)
		return NULL;
	cache->size = size;
	cache->count = count;
	spin_lock_init(&cache->lock);

	cache->cache = kmalloc(size * count, GFP_KERNEL);
	if (!cache->cache)
		goto err_cache;
	cache->map = kzalloc(BITS_TO_LONGS(count) * sizeof(long), GFP_KERNEL);
	if (!cache->map)
		goto err_map;

	return cache;
err_map:
	kfree(cache->cache);
err_cache:
	kfree(cache);
	cache = NULL;
	return cache;
}

static void __maybe_unused epdc_cache_destroy(struct epdc_cache *cache)
{
	kfree((const void *)(cache->map));
	kfree(cache->cache);
	kfree(cache);
}

#endif
