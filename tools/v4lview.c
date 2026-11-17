#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <poll.h>
#include <math.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <linux/fb.h>
#include <linux/kd.h>
#define NB 3
#define MAXCAM 2
struct cam { int fd, w, h, x0, rw, rh; void *buf[NB]; size_t len[NB]; unsigned frames; int exp, gain; int rg, bg; unsigned char *bb; };
static volatile int stop;
static unsigned char lut[1024];
static struct cam cams[MAXCAM];
static int ncam;
static unsigned char *fbp;
static int fbw, fbh, fbbpp, fbline;
static void onsig(int s) { (void)s; stop = 1; }
static int xioctl(int fd, unsigned long r, void *a) { int e; do e = ioctl(fd, r, a); while (e < 0 && errno == EINTR); return e; }
static void setctl(struct cam *c, unsigned id, int v) { struct v4l2_control ctl = { .id = id, .value = v }; xioctl(c->fd, VIDIOC_S_CTRL, &ctl); }
static int open_cam(struct cam *c, const char *path, int W, int H)
{
	struct v4l2_format f; struct v4l2_requestbuffers rq; int i, type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	c->fd = open(path, O_RDWR);
	if (c->fd < 0) { perror(path); return -1; }
	memset(&f, 0, sizeof f); f.type = type; f.fmt.pix_mp.width = W; f.fmt.pix_mp.height = H;
	f.fmt.pix_mp.pixelformat = v4l2_fourcc('R', 'G', '1', '0'); f.fmt.pix_mp.num_planes = 1; f.fmt.pix_mp.field = V4L2_FIELD_NONE;
	if (xioctl(c->fd, VIDIOC_S_FMT, &f) < 0) { perror("S_FMT"); return -1; }
	c->w = f.fmt.pix_mp.width; c->h = f.fmt.pix_mp.height;
	memset(&rq, 0, sizeof rq); rq.count = NB; rq.type = type; rq.memory = V4L2_MEMORY_MMAP;
	if (xioctl(c->fd, VIDIOC_REQBUFS, &rq) < 0) { perror("REQBUFS"); return -1; }
	for (i = 0; i < NB; i++) {
		struct v4l2_buffer b; struct v4l2_plane pl[VIDEO_MAX_PLANES];
		memset(&b, 0, sizeof b); memset(pl, 0, sizeof pl);
		b.type = type; b.memory = V4L2_MEMORY_MMAP; b.index = i; b.m.planes = pl; b.length = 1;
		if (xioctl(c->fd, VIDIOC_QUERYBUF, &b) < 0) { perror("QUERYBUF"); return -1; }
		c->len[i] = pl[0].length;
		c->buf[i] = mmap(NULL, pl[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, c->fd, pl[0].m.mem_offset);
		if (c->buf[i] == MAP_FAILED) { perror("mmap"); return -1; }
		if (xioctl(c->fd, VIDIOC_QBUF, &b) < 0) { perror("QBUF"); return -1; }
	}
	c->rg = c->bg = 256;
	return 0;
}
static void render(struct cam *c, const unsigned short *a, int noae, int emax, int target)
{
	int W = c->w, bw = W / 2, bh = c->h / 2, dx, dy, sx, sy;
	long sR = 0, sG = 0, sB = 0, n = 0;
	{ static unsigned short *loc; static size_t locsz; size_t need = (size_t)W * c->h * 2;
	  if (locsz < need) { free(loc); loc = malloc(need); locsz = loc ? need : 0; }
	  if (loc) { memcpy(loc, a, need); a = loc; } }
	for (sy = 0; sy < bh; sy += 4)
		for (sx = 0; sx < bw; sx += 4) {
			const unsigned short *p = a + (sy * 2) * W + sx * 2;
			int B = p[0] - 64, G = ((p[1] + p[W]) >> 1) - 64, R = p[W + 1] - 64;
			if (B < 0) B = 0;
			if (G < 0) G = 0;
			if (R < 0) R = 0;
			sB += B; sG += G; sR += R; n++;
		}
	for (dy = 0; dy < c->rh; dy++) {
		int by = dy * bh / c->rh;
		for (dx = 0; dx < c->rw; dx++) {
			int bx = dx * bw / c->rw;
			const unsigned short *p = a + (by * 2) * W + bx * 2;
			int B = p[0] - 64, G = ((p[1] + p[W]) >> 1) - 64, R = p[W + 1] - 64;
			unsigned char r8, g8, b8;
			unsigned char *o = c->bb + ((long)dy * c->rw + dx) * (fbbpp / 8);
			if (B < 0) B = 0;
			if (G < 0) G = 0;
			if (R < 0) R = 0;
			R = (R * c->rg) >> 8; B = (B * c->bg) >> 8;
			if (R > 1023) R = 1023;
			if (G > 1023) G = 1023;
			if (B > 1023) B = 1023;
			r8 = lut[R]; g8 = lut[G]; b8 = lut[B];
			if (fbbpp == 32) { o[0] = b8; o[1] = g8; o[2] = r8; o[3] = 0; }
			else *(unsigned short *)o = ((r8 >> 3) << 11) | ((g8 >> 2) << 5) | (b8 >> 3);
		}
	}
	for (dy = 0; dy < c->rh; dy++)
		memcpy(fbp + (long)dy * fbline + (long)c->x0 * (fbbpp / 8), c->bb + (long)dy * c->rw * (fbbpp / 8), (long)c->rw * (fbbpp / 8));
	if (n && sR > 0 && sB > 0) {
		int rg = (int)(256.0 * sG / sR), bg = (int)(256.0 * sG / sB);
		if (rg < 128) rg = 128;
		if (rg > 1024) rg = 1024;
		if (bg < 128) bg = 128;
		if (bg > 1024) bg = 1024;
		c->rg = (c->rg * 7 + rg) / 8; c->bg = (c->bg * 7 + bg) / 8;
	}
	if (!noae && n && !(c->frames % 2)) {
		double mean = (double)sG / n, r = target / (mean + 1.0);
		if (r > 1.15 || r < 0.87) {
			double tot = (double)c->exp * c->gain / 16.0; int e, g;
			tot *= pow(r, 0.6);
			if (tot < 16) tot = 16;
			if (tot <= emax) { e = (int)tot; g = 16; }
			else { e = emax; g = (int)(tot * 16.0 / emax); if (g > 800) g = 800; }
			if (e != c->exp) setctl(c, V4L2_CID_EXPOSURE, e);
			if (g != c->gain) setctl(c, V4L2_CID_GAIN, g);
			c->exp = e; c->gain = g;
		}
	}
}
int main(int argc, char **argv)
{
	int W = getenv("W") ? atoi(getenv("W")) : 1280, H = getenv("H") ? atoi(getenv("H")) : 720;
	int secs = getenv("SECS") ? atoi(getenv("SECS")) : 30, noae = getenv("NOAE") != NULL;
	int emax = getenv("EMAX") ? atoi(getenv("EMAX")) : 12800;
	int target = getenv("TARGET") ? atoi(getenv("TARGET")) : 170;
	const char *devs[MAXCAM] = { "/dev/video0", "/dev/video8" };
	struct fb_var_screeninfo vi; struct fb_fix_screeninfo fi;
	int fb, tty, i, type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	struct timespec t0, t1;
	if (argc > 1) { ncam = argc - 1 > MAXCAM ? MAXCAM : argc - 1; for (i = 0; i < ncam; i++) devs[i] = argv[i + 1]; }
	else ncam = 1;
	for (i = 0; i < 1024; i++) { double v = pow(i / 959.0, 1 / 2.2); lut[i] = v > 1 ? 255 : (unsigned char)(v * 255); }
	fb = open("/dev/fb0", O_RDWR);
	if (fb < 0 || ioctl(fb, FBIOGET_VSCREENINFO, &vi) || ioctl(fb, FBIOGET_FSCREENINFO, &fi)) { perror("fb0"); return 1; }
	fbw = vi.xres; fbh = vi.yres; fbbpp = vi.bits_per_pixel; fbline = fi.line_length;
	if (fbbpp != 32 && fbbpp != 16) { fprintf(stderr, "unsupported fb bpp %d\n", fbbpp); return 1; }
	fbp = mmap(NULL, fi.smem_len, PROT_READ | PROT_WRITE, MAP_SHARED, fb, 0);
	memset(fbp, 0, fi.smem_len);
	tty = open("/dev/tty0", O_RDWR);
	if (tty >= 0) ioctl(tty, KDSETMODE, KD_GRAPHICS);
	signal(SIGINT, onsig); signal(SIGTERM, onsig);
	for (i = 0; i < ncam; i++) {
		struct cam *c = &cams[i];
		if (open_cam(c, devs[i], W, H)) return 1;
		c->rw = fbw / ncam; c->rh = c->rw * c->h / c->w;
		if (c->rh > fbh) { c->rh = fbh; c->rw = c->rh * c->w / c->h; }
		c->x0 = i * (fbw / ncam); c->bb = calloc((size_t)c->rw * c->rh, fbbpp / 8);
		c->exp = emax < 6400 ? emax : 6400; c->gain = 16;
		setctl(c, V4L2_CID_EXPOSURE, c->exp); setctl(c, V4L2_CID_GAIN, c->gain);
		if (xioctl(c->fd, VIDIOC_STREAMON, &type) < 0) { perror("STREAMON"); return 1; }
		fprintf(stderr, "%s: %dx%d -> %dx%d on screen\n", devs[i], c->w, c->h, c->rw, c->rh);
	}
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while (!stop) {
		struct pollfd p[MAXCAM];
		for (i = 0; i < ncam; i++) { p[i].fd = cams[i].fd; p[i].events = POLLIN; p[i].revents = 0; }
		if (poll(p, ncam, 2000) <= 0) { fprintf(stderr, "timeout: no frames\n"); break; }
		for (i = 0; i < ncam; i++) {
			struct v4l2_buffer b; struct v4l2_plane pl[VIDEO_MAX_PLANES];
			if (!(p[i].revents & POLLIN)) continue;
			memset(&b, 0, sizeof b); memset(pl, 0, sizeof pl);
			b.type = type; b.memory = V4L2_MEMORY_MMAP; b.m.planes = pl; b.length = 1;
			if (xioctl(cams[i].fd, VIDIOC_DQBUF, &b) < 0) continue;
			if (!(b.flags & V4L2_BUF_FLAG_ERROR)) render(&cams[i], cams[i].buf[b.index], noae, emax, target);
			cams[i].frames++;
			xioctl(cams[i].fd, VIDIOC_QBUF, &b);
		}
		clock_gettime(CLOCK_MONOTONIC, &t1);
		if (t1.tv_sec - t0.tv_sec >= secs) break;
	}
	clock_gettime(CLOCK_MONOTONIC, &t1);
	for (i = 0; i < ncam; i++) {
		double dt = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
		xioctl(cams[i].fd, VIDIOC_STREAMOFF, &type);
		fprintf(stderr, "%s: %u frames, %.1f fps, exp=%d gain=%d wb=%d/%d\n", devs[i], cams[i].frames, cams[i].frames / dt, cams[i].exp, cams[i].gain, cams[i].rg, cams[i].bg);
	}
	if (tty >= 0) ioctl(tty, KDSETMODE, KD_TEXT);
	return 0;
}
