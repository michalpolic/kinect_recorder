
#include "io.hpp"


namespace kinectrec {

	void io::flipX(F *frame) {
		int w = frame->width;
		int h = frame->height;
		unsigned int *frame_data = (unsigned int*)frame->data;   // something with 4 bytes to simplify iterations
		unsigned int *buffer = (unsigned int*)malloc(frame->width * sizeof(unsigned int));
		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++)
				buffer[x] = frame_data[y * w + x];
			for (int x = 0; x < w; x++)
				frame_data[(y + 1) * w - x - 1] = buffer[x];
		}
	}

	void io::saveJpeg(const F *frame, const std::string file_path) {
		// init
		unsigned char *rgb_buffer = (unsigned char*)frame->data;
		long unsigned int _jpegSize = 0;
		unsigned char* _compressedImage = NULL;			// < Memory is allocated by tjCompress2 if _jpegSize == 0
		tjhandle _jpegCompressor = tjInitCompress();

		// compress 
		tjCompress2(_jpegCompressor, rgb_buffer, frame->width, 0, frame->height, TJPF_BGRX,
			&_compressedImage, &_jpegSize, TJSAMP_444, 98, TJFLAG_FASTDCT);

		// save 
		FILE *file = fopen(file_path.c_str(), "w+b");
		if (!file)
			std::cerr << "Could not open JPEG file: " << strerror(errno);
		if (fwrite(_compressedImage, _jpegSize, 1, file) < 1)
			std::cerr << "Could not write JPEG file: " << strerror(errno);
		fclose(file);

		// clear
		tjDestroy(_jpegCompressor);
		tjFree(_compressedImage);		//to free the memory allocated by TurboJPEG
	}

	void io::write_png_file(const char *filename, F *depth)
	{
		FILE *fp = fopen(filename, "w+b");
		if (!fp) abort();

		png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
		if (!png) abort();

		png_infop info = png_create_info_struct(png);
		if (!info) abort();

		if (setjmp(png_jmpbuf(png))) abort();

		png_init_io(png, fp);

		// Output is 8bit depth, RGBA format.
		png_set_IHDR(
			png,
			info,
			depth->width, depth->height,
			16,
			PNG_COLOR_TYPE_GRAY,
			PNG_INTERLACE_NONE,
			PNG_COMPRESSION_TYPE_DEFAULT,
			PNG_FILTER_TYPE_DEFAULT
		);
		png_write_info(png, info);

		// To remove the alpha channel for PNG_COLOR_TYPE_RGB format,
		// Use png_set_filler().
		//png_set_filler(png, 0, PNG_FILLER_AFTER);



		// reformat depth data to 16bit
		png_bytepp row_pointers;
		row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * depth->height);
		const float *depth_data = ((float*)depth->data);

		for (int y = 0; y < depth->height; y++) {
			row_pointers[y] = (png_byte*)malloc(png_get_rowbytes(png, info));
			png_bytep row = row_pointers[y];

			for (int x = 0; x < depth->width; x++) {
				png_bytep px = &(row[x * 2]);

				int i = y * depth->width + x;
				unsigned short d;
				if (depth_data[i] < (std::numeric_limits<unsigned short>::max)())
					d = static_cast<unsigned short>(std::round(depth_data[i]));
				else
					d = 0;

				px[1] = (d >> 0);
				px[0] = (d >> 8);
			}
		}


		png_write_image(png, row_pointers);   // png_uint_16
		png_write_end(png, NULL);

		fclose(fp);


		// free png buffer 
		for (int y = 0; y < depth->height; y++) {
			free(row_pointers[y]);
		}
		free(row_pointers);

		// free auxiliary png info
		if (png && info)
			png_destroy_write_struct(&png, &info);
	}

	void io::saveImgs(std::string base_path, F *rgb, F *depth, F *ir, F *registered, F *bigdepth) {
		using namespace std::chrono;
		system_clock::time_point today = system_clock::now();
		milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
		std::string timestamp = std::to_string(ms.count());

		//std::string base_path = IO_BASE_PATH;
		if (CreateDirectory(base_path.c_str(), NULL) ||
			ERROR_ALREADY_EXISTS == GetLastError())
		{
			// save rgb image
			if (rgb != NULL)
			{
				if (CreateDirectory(std::string(base_path + "/rgb").c_str(), NULL) ||
					ERROR_ALREADY_EXISTS == GetLastError())
				{
					std::string rgb_file_path(base_path + "/rgb/" + timestamp + ".jpg");
					//unsigned char *rgb_buffer = (unsigned char*)rgb->data;
					//tjSaveImage(rgb_file_path.c_str(), rgb_buffer, rgb->width, 0, rgb->height, TJPF_BGRX, TJFLAG_ACCURATEDCT);
					io::saveJpeg(rgb, rgb_file_path);
				}
			}

			//// save ir image
			if (ir != NULL)
			{
				if (CreateDirectory(std::string(base_path + "/ir").c_str(), NULL) ||
					ERROR_ALREADY_EXISTS == GetLastError())
				{
					std::string ir_file_path(base_path + "/ir/" + timestamp + ".png");
					write_png_file(ir_file_path.c_str(), ir);
				}
			}

			// save depth image
			if (depth != NULL)
			{
				if (CreateDirectory(std::string(base_path + "/depth").c_str(), NULL) ||
					ERROR_ALREADY_EXISTS == GetLastError())
				{
					std::string depth_file_path(base_path + "/depth/" + timestamp + ".png");
					write_png_file(depth_file_path.c_str(), depth);
				}
			}

			// save registered image
			if (registered != NULL)
			{
				if (CreateDirectory(std::string(base_path + "/registered").c_str(), NULL) ||
					ERROR_ALREADY_EXISTS == GetLastError())
				{
					std::string registered_file_path(base_path + "/registered/" + timestamp + ".bmp");
					unsigned char *registered_buffer = (unsigned char*)registered->data;
					tjSaveImage(registered_file_path.c_str(), registered_buffer, rgb->width, 0, rgb->height, TJPF_BGRX, TJFLAG_ACCURATEDCT);
				}
			}

			// save depth image
			if (bigdepth != NULL)
			{
				if (CreateDirectory(std::string(base_path + "/bigdepth").c_str(), NULL) ||
					ERROR_ALREADY_EXISTS == GetLastError())
				{
					std::string bigdepth_file_path(base_path + "/bigdepth/" + timestamp + ".png");
					write_png_file(bigdepth_file_path.c_str(), bigdepth);
				}
			}
		}
	}

}