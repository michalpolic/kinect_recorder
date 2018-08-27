
#ifndef IO_H
#define IO_H

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <fstream>
#include <string> 
#include <sstream>
#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
#include <windows.h>

#include "png.h"
#include "turbojpeg.h"

using F = libfreenect2::Frame;


namespace kinectrec{

	class io {
	public:
		static void flipX(F *frame);
		static void saveJpeg(const F *frame, const std::string file_path);
		static void write_png_file(const char *filename, F *depth);
		static void saveImgs(std::string base_path, F *rgb, F *depth, F *ir, F *registered, F *bigdepth);
	};

}

#endif