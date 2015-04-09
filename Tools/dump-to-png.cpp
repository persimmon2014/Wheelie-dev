#include "libroad/libroad_common.hpp"
#include <png.h>
#include <string>
#include <cstdlib>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <cassert>
#include <iostream>

static void abort_(const char * s, ...)
{
	va_list args;
	va_start(args, s);
	vfprintf(stderr, s, args);
	fprintf(stderr, "\n");
	va_end(args);
	abort();
}

// writes an 8-bit RGB png from an BGRA buffer
static void write_png_file(const unsigned char *pix, const int w, const int h, const std::string &fname)
{
    png_byte color_type = PNG_COLOR_TYPE_RGB;
    png_byte bit_depth = 8;

    png_infop info_ptr;

	/* create file */
    FILE *fp = fopen(fname.c_str(), "wb");
	if (!fp)
		abort_("[write_png_file] File %s could not be opened for writing", fname.c_str());

	/* initialize stuff */
	png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

	if (!png_ptr)
		abort_("[write_png_file] png_create_write_struct failed");

	info_ptr = png_create_info_struct(png_ptr);
	if (!info_ptr)
		abort_("[write_png_file] png_create_info_struct failed");

	if (setjmp(png_jmpbuf(png_ptr)))
		abort_("[write_png_file] Error during init_io");

	png_init_io(png_ptr, fp);

	/* write header */
	if (setjmp(png_jmpbuf(png_ptr)))
		abort_("[write_png_file] Error during writing header");

	png_set_IHDR(png_ptr, info_ptr, w, h,
		     bit_depth, color_type, PNG_INTERLACE_NONE,
		     PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

	png_write_info(png_ptr, info_ptr);

	/* write bytes */
	if (setjmp(png_jmpbuf(png_ptr)))
		abort_("[write_png_file] Error during writing bytes");

    png_set_compression_level(png_ptr,
                              Z_BEST_COMPRESSION);

    png_bytep row = (png_bytep)malloc(sizeof(png_byte)*3*w);
    for(int i = h-1; i >= 0; --i)
    {
        png_bytep row_pointer = (png_byte*)pix+i*4*w;
        for(int p = 0; p < w; ++p)
        {
            row[3*p+0] = row_pointer[4*p+2];
            row[3*p+1] = row_pointer[4*p+1];
            row[3*p+2] = row_pointer[4*p+0];
        }
        png_write_row(png_ptr, row);
    }
    free(row);

	/* end write */
	if (setjmp(png_jmpbuf(png_ptr)))
		abort_("[write_png_file] Error during end of write");

	png_write_end(png_ptr, NULL);

    fclose(fp);
}

struct header
{
    char magic[5];
    int  w;
    int  h;
    int  bpp;
    char format[5];
};

static void convert_dump(const std::string &out_file, const std::string &in_file)
{
    int fin = open(in_file.c_str(), O_RDONLY);
    assert(fin != -1);
    struct stat file_stat;
    fstat(fin, &file_stat);

    void *newbuff = mmap(0, file_stat.st_size, PROT_READ, MAP_SHARED, fin, 0);
    assert(newbuff && newbuff != MAP_FAILED);
    close(fin);

    const header *head = reinterpret_cast<const header*>(newbuff);
    assert(strncmp(head->magic, "DUMP", 5) == 0);
    assert(strncmp(head->format, "BGRA", 5) == 0);
    assert(head->bpp == 1);

    write_png_file(static_cast<unsigned char*>(newbuff) + sizeof(header),
                   head->w, head->h, out_file.c_str());

    munmap(newbuff, file_stat.st_size);
    std::cout << "Wrote " << out_file << std::endl;
}

int main(int argc, char *argv[])
{
    if(argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input dumpfile> " << std::endl;
        return 1;
    }

    convert_dump(bf::path(argv[1]).replace_extension("png").string(), argv[1]);

    return 0;
}
