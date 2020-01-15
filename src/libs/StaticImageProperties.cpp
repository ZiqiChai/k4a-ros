#include <cmath>

#include "StaticImageProperties.h"

void sen::ColorizeDepthImage(const k4a::image &depthImage,
	DepthPixelVisualizationFunction visualizationFn,
	std::pair<uint16_t, uint16_t> expectedValueRange,
	std::vector<Pixel> *buffer)
{
	const k4a_image_format_t imageFormat = depthImage.get_format();
	if (imageFormat != K4A_IMAGE_FORMAT_DEPTH16 && imageFormat != K4A_IMAGE_FORMAT_IR16)

	{
		throw std::logic_error("Attempted to colorize a non-depth image!");
	}

	const int width = depthImage.get_width_pixels();
	const int height = depthImage.get_height_pixels();

	buffer->resize(static_cast<size_t>(width * height));

	const uint16_t *depthData = reinterpret_cast<const uint16_t *>(depthImage.get_buffer());
	for (int h = 0; h < height; ++h)
	{
		for (int w = 0; w < width; ++w)
		{
			const size_t currentPixel = static_cast<size_t>(h * width + w);
			(*buffer)[currentPixel] = visualizationFn(depthData[currentPixel],
				expectedValueRange.first,
				expectedValueRange.second);
		}
	}
}

bool sen::ColorizeDepthImage(const k4a_image_t depth_image,
	DepthPixelVisualizationFunction visualizationFn,
	std::pair<uint16_t, uint16_t> expectedValueRange,
	std::vector<Pixel> *buffer)
{
	const k4a_image_format_t imageFormat = k4a_image_get_format(depth_image);
	if (imageFormat != K4A_IMAGE_FORMAT_DEPTH16 && imageFormat != K4A_IMAGE_FORMAT_IR16)
	{
		throw std::logic_error("Attempted to colorize a non-depth image!");
		return false;
	}

	const int width = k4a_image_get_width_pixels(depth_image);
	const int height = k4a_image_get_height_pixels(depth_image);;

	buffer->resize(static_cast<size_t>(width * height));

	const uint16_t *depthData = reinterpret_cast<uint16_t *>(k4a_image_get_buffer(depth_image));
	for (int h = 0; h < height; ++h)
	{
		for (int w = 0; w < width; ++w)
		{
			const size_t currentPixel = static_cast<size_t>(h * width + w);
			(*buffer)[currentPixel] = visualizationFn(depthData[currentPixel],
				expectedValueRange.first,
				expectedValueRange.second);
		}
	}
	return true;
}


void sen::GetCV_DepthImage(
	const k4a::image &depthImage,
	std::vector<Pixel> *buffer)
{
	const k4a_image_format_t imageFormat = depthImage.get_format();
	if (imageFormat != K4A_IMAGE_FORMAT_DEPTH16 && imageFormat != K4A_IMAGE_FORMAT_IR16)

	{
		throw std::logic_error("Attempted to colorize a non-depth image!");
	}

	const int width = depthImage.get_width_pixels();
	const int height = depthImage.get_height_pixels();

	buffer->resize(static_cast<size_t>(width * height));

	const uint16_t *depthData = reinterpret_cast<const uint16_t *>(depthImage.get_buffer());
	for (int h = 0; h < height; ++h)
	{
		for (int w = 0; w < width; ++w)
		{
			const size_t currentPixel = static_cast<size_t>(h * width + w);
			// All color channels are set the same (image is greyscale)
			(*buffer)[currentPixel] =Pixel{
				static_cast<uint8_t>(depthData[currentPixel]),
				static_cast<uint8_t>(depthData[currentPixel]),
				static_cast<uint8_t>(depthData[currentPixel]),
				static_cast<uint8_t>(depthData[currentPixel])
			};
		}
	}
}


void sen::GetCV_DepthImage(
	const k4a::image &depthImage,
	std::vector<uint16_t> *buffer)
{
	const k4a_image_format_t imageFormat = depthImage.get_format();
	if (imageFormat != K4A_IMAGE_FORMAT_DEPTH16 && imageFormat != K4A_IMAGE_FORMAT_IR16)

	{
		throw std::logic_error("Attempted to colorize a non-depth image!");
	}

	const int width = depthImage.get_width_pixels();
	const int height = depthImage.get_height_pixels();

	buffer->resize(static_cast<size_t>(width * height));

	const uint16_t *depthData = reinterpret_cast<const uint16_t *>(depthImage.get_buffer());
	for (int h = 0; h < height; ++h)
	{
		for (int w = 0; w < width; ++w)
		{
			const size_t currentPixel = static_cast<size_t>(h * width + w);
			// All color channels are set the same (image is greyscale)
			(*buffer)[currentPixel] =depthData[currentPixel];
		}
	}
}