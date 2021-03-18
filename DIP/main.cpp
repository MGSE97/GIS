#include <stdio.h>
#include <stdlib.h>

#include <cmath>

#include <opencv2/opencv.hpp>

#define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H true
#include <proj_api.h>

#include "defines.h"
#include "utils.h"
#include "main.h"

#define STEP1_WIN_NAME "Heightmap"
#define STEP2_WIN_NAME "Edges"
#define ZOOM           1


struct LidarRanges {
    float min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
    float delta_x_, delta_y_, delta_z_;
    char* filename_;
    int type_ = 0x1F5711;
    int offset_ = 0;
    int base_type_ = 0x1F5711;

    LidarRanges(const char* filename, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, float delta_x, float delta_y, float delta_z)
        : filename_((char*)filename), min_x_(min_x), max_x_(max_x), min_y_(min_y), max_y_(max_y), min_z_(min_z), max_z_(max_z), delta_x_(delta_x), delta_y_(delta_y), delta_z_(delta_z)
    {
    }
};

struct MouseProbe {
    cv::Mat & heightmap_8uc1_img_;
    cv::Mat & heightmap_show_8uc3_img_;
    cv::Mat & edgemap_8uc1_img_;
    LidarRanges& lidar_ranges_;

    MouseProbe( cv::Mat & heightmap_8uc1_img, cv::Mat & heightmap_show_8uc3_img, cv::Mat & edgemap_8uc1_img, LidarRanges& lidar_ranges)
     : heightmap_8uc1_img_( heightmap_8uc1_img ), heightmap_show_8uc3_img_( heightmap_show_8uc3_img ), edgemap_8uc1_img_( edgemap_8uc1_img ), lidar_ranges_(lidar_ranges)
    {
    }
};

// variables

// function declarations
void flood_fill( cv::Mat & src_img, cv::Mat & dst_img, const int x, const int y );



bool is_safe(cv::Mat& image, int x, int y)
{
    return !(x < 0 || x >= image.cols || y < 0 || y >= image.rows);
}

unsigned char get_safe_value(cv::Mat& image, int x, int y)
{
    if (x < 0 || x >= image.cols || y < 0 || y >= image.rows)
        return 0;

    return image.at<unsigned char>(y, x);
}

/**
 * Perform flood fill from the specified point (x, y) for the neighborhood points if they contain the same value,
 * as the one that came in argument 'value'. Function recursicely call itself for its 4-neighborhood.
 * 
 * edgemap_8uc1_img - image, in which we perform flood filling
 * heightmap_show_8uc3_img - image, in which we display the filling
 * value - value, for which we'll perform flood filling
 */
void fill_step(cv::Mat & edgemap_8uc1_img, cv::Mat & heightmap_show_8uc3_img, const int x, const int y, const unsigned char value, std::queue<cv::Vec2i>& queue) {
    int dx = 0, dy = 0;

    heightmap_show_8uc3_img.at<cv::Vec3b>(y, x).val[2] = 255;

    dx = x - 1; dy = y;
    if (is_safe(edgemap_8uc1_img, dx, dy) && edgemap_8uc1_img.at<unsigned char>(dy, dx) == value)
        queue.push(cv::Vec2i(dx, dy));
    
    dx = x + 1; dy = y;
    if (is_safe(edgemap_8uc1_img, dx, dy) && edgemap_8uc1_img.at<unsigned char>(dy, dx) == value)
        queue.push(cv::Vec2i(dx, dy));
    
    dx = x; dy = y - 1;
    if (is_safe(edgemap_8uc1_img, dx, dy) && edgemap_8uc1_img.at<unsigned char>(dy, dx) == value)
        queue.push(cv::Vec2i(dx, dy));

    dx = x; dy = y + 1;
    if (is_safe(edgemap_8uc1_img, dx, dy) && edgemap_8uc1_img.at<unsigned char>(dy, dx) == value)
        queue.push(cv::Vec2i(dx, dy));
} //fill_step


/**
 * Perform flood fill from the specified point (x, y). The function remembers the value at the coordinate (x, y)
 * and fill the neighborhood using 'fill_step' function so long as the value in the neighborhood points are the same.
 * Execute the fill on a temporary image to prevent the original image from being repainted.

 * edgemap_8uc1_img - image, in which we perform flood filling
 * heightmap_show_8uc3_img - image, in which we display the filling
 */
void flood_fill( cv::Mat & edgemap_8uc1_img, cv::Mat & heightmap_show_8uc3_img, const int x, const int y ) {
    cv::Mat tmp_edgemap_8uc1_img;
    
    std::queue<cv::Vec2i> queue;
    fill_step(edgemap_8uc1_img, heightmap_show_8uc3_img, x, y, edgemap_8uc1_img.at<unsigned char>(y, x), queue);

    while (!queue.empty())
    {
        cv::Vec2i point = queue.front();
        queue.pop();
        if(heightmap_show_8uc3_img.at<cv::Vec3b>(point[1], point[0]).val[2] != 255)
            fill_step(edgemap_8uc1_img, heightmap_show_8uc3_img, point[0], point[1], edgemap_8uc1_img.at<unsigned char>(point[1], point[0]), queue);
    }
} //flood_fill



struct lidar_data {
    float x, y, z;
    int type; // Magic number of reflections
};
/**
 * Find the minimum and maximum coordinates in the file.
 * Note that the file is the S-JTSK coordinate system.
 */
#define BUFFER_SIZE 1024
void get_min_max( const char *filename, float *a_min_x, float *a_max_x, float *a_min_y, float *a_max_y, float *a_min_z, float *a_max_z ) {
    FILE* f = fopen(filename,"rb");
    if (f == NULL)
    {
        printf("%s not found!\n", filename);
        return exit(-1);
    }

    float
        min_x = INT32_MAX,
        min_y = INT32_MAX,
        min_z = INT32_MAX,
        max_x = INT32_MIN,
        max_y = INT32_MIN,
        max_z = INT32_MIN;
    int items_c, i;
    lidar_data items[BUFFER_SIZE], item;

    while ((items_c = fread(&items, sizeof(lidar_data), BUFFER_SIZE, f)) != 0)
    {
        for (i = 0; i < items_c; i++)
        {
            item = items[i];
            if (item.x < min_x)
                min_x = item.x;
            if (item.x > max_x)
                max_x = item.x;

            if (item.y < min_y)
                min_y = item.y;
            if (item.y > max_y)
                max_y = item.y;

            if (item.z < min_z)
                min_z = item.z;
            if (item.z > max_z)
                max_z = item.z;
        }
    }

    fclose (f);

    *a_min_x = min_x;
    *a_min_y = min_y;
    *a_min_z = min_z;
    *a_max_x = max_x;
    *a_max_y = max_y;
    *a_max_z = max_z;
}


/**
 * Fill the image by data from lidar.
 * All lidar points are stored in a an array that has the dimensions of the image. Then the pixel is assigned
 * a value as an average value range from at the corresponding array element. However, with this simple data access, you will lose data precission.
 * filename - file with binarny data
 * img - input image
 */
void fill_image( const char *filename, cv::Mat & heightmap_8uc1_img, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, int type ) {
    float delta_x, delta_y, delta_z;
    int x, y, z;

    // zjistime sirku a vysku obrazu
    delta_x = round( max_x - min_x + 0.5f );
    delta_y = round( max_y - min_y + 0.5f );
    delta_z = round( max_z - min_z + 0.5f );

    // 1:
    // We allocate helper arrays, in which we store values from the lidar
    // and the number of these values for each pixel
    cv::Mat sum_heights = cv::Mat::zeros(cvSize(cvRound(delta_x + 0.5f), cvRound(delta_y + 0.5f)), CV_32FC1);
    cv::Mat count_heights = cv::Mat::zeros(cvSize(cvRound(delta_x + 0.5f), cvRound(delta_y + 0.5f)), CV_8UC1);

    // 2:
    // go through the file and assign values to the field
    // beware that in S-JTSK the beginning of the co-ordinate system is at the bottom left,
    // while in the picture it is top left

    FILE* f = fopen(filename, "rb");
    if (f == NULL)
    {
        printf("%s not found!\n", filename);
        return exit(-1);
    }
    int items_c, i, total = 0, mask;
    lidar_data items[BUFFER_SIZE], item;

    // Get heights
    while ((items_c = fread(&items, sizeof(lidar_data), BUFFER_SIZE, f)) != 0)
    {
        for (i = 0; i < items_c; i++)
        {
            item = items[i];
            // 0000 0111
            mask = 1 << item.type;
            if ((type & mask) != mask)
                continue;

            x = round(item.x - min_x);
            y = round(item.y - min_y);
            z = round(item.z - min_z);

            sum_heights.at<float>(y, x) += z;
            count_heights.at<unsigned char>(y, x) += 1;
        }
    }

    fclose(f);

    // 3:
    // assign values from the helper field into the image
    // Set to image
    for (int y = 0; y < heightmap_8uc1_img.rows; y++)
    {
        for (int x = 0; x < heightmap_8uc1_img.cols; x++)
        {
            // Average
            float height = sum_heights.at<float>(y, x) / (float)count_heights.at<unsigned char>(y, x);

            // Set to image
            heightmap_8uc1_img.at<unsigned char>(y, x) = (height / delta_z) * 255.f;
        }
    }
}

// https://answers.opencv.org/question/176494/how-to-calculate-the-median-of-pixel-values-in-opencv-python/?sort=oldest
int static Median(const cv::Mat& img, int nVals) {
    // COMPUTE HISTOGRAM OF SINGLE CHANNEL MATRIX
    float range[] = { 0, nVals };
    const float* histRange = { range };
    bool uniform = true;
    bool accumulate = false;
    int channels[] = { 0 };
    cv::Mat hist;
    calcHist(&img, 1, channels, cv::Mat(), hist, 1, &nVals, &histRange, uniform, accumulate);

    // COMPUTE CUMULATIVE DISTRIBUTION FUNCTION (CDF)
    cv::Mat cdf;
    hist.copyTo(cdf);
    for (int i = 1; i < nVals - 1; i++) {
        cdf.at<float>(i) += cdf.at<float>(i - 1);
    }
    cdf /= static_cast<float>(img.total());

    // COMPUTE MEDIAN
    int medianVal = 0;
    for (int i = 0; i < nVals - 1; i++) {
        if (cdf.at<float>(i) >= 0.5) { medianVal = i;  break; }
    }

    cdf.deallocate();
    hist.deallocate();

    return medianVal;
}

// https://www.pyimagesearch.com/2015/04/06/zero-parameter-automatic-canny-edge-detection-with-python-and-opencv/
void static AutoCanny(const cv::Mat& src, cv::Mat& edges, float sigma = 0.33)
{
    //compute the median of the single channel pixel intensities
    const auto v = Median(src, 256);

    //apply automatic Canny edge detection using the computed median
    const auto lower = int(MAX(0, (1.0 - sigma) * v));
    const auto upper = int(MIN(255, (1.0 + sigma) * v));
    cv::Canny(src, edges, lower, upper);
}

void make_edges(const cv::Mat & src_8uc1_img, cv::Mat & edgemap_8uc1_img ) {
    cv::Canny(src_8uc1_img, edgemap_8uc1_img, 1, 80 );
    //AutoCanny(src_8uc1_img, edgemap_8uc1_img);
}


/**
 * Transforms the image so it contains only two values.
 * Threshold may be set experimentally.
 */
void binarize_image( cv::Mat & src_8uc1_img ) {
    for (int y = 0; y < src_8uc1_img.rows; y++)
        for (int x = 0; x < src_8uc1_img.cols; x++)
            src_8uc1_img.at<unsigned char>(y, x) = src_8uc1_img.at<unsigned char>(y, x) > 128 ? 255 : 0;
}

void dilatate_image(cv::Mat& src_8uc1_img, int mask_width, int mask_height)
{
    int y, x, dx, dy, sx, sy;
    bool fill;
    int minx = -(mask_width / 2), maxx = mask_width / 2 + 1;
    int miny = -(mask_height / 2), maxy = mask_height / 2 + 1;
    
    cv::Mat result;
    src_8uc1_img.copyTo(result);

    for (y = 0; y < src_8uc1_img.rows; y++)
        for (x = 0; x < src_8uc1_img.cols; x++)
        {
            // Check center
            fill = get_safe_value(src_8uc1_img, x, y) == 255;
         
            // Fill mask
            if(fill)
                for (dy = 0; dy < mask_height; dy++)
                {
                    sy = dy + y;
                    for (dx = 0; dx < mask_height; dx++)
                    {
                        sx = dx + x;
                        if(is_safe(src_8uc1_img, sx, sy))
                            result.at<unsigned char>(sy, sx) = 255;
                    }
                }
        }

    result.copyTo(src_8uc1_img);
}

void erode_image(cv::Mat& src_8uc1_img, int mask_width, int mask_height)
{
    int y, x, dx, dy, sx, sy;
    bool empty;
    int minx = -(mask_width / 2), maxx = mask_width / 2 + 1;
    int miny = -(mask_height / 2), maxy = mask_height / 2 + 1;
    cv::Mat result;
    src_8uc1_img.copyTo(result);

    for (y = 0; y < src_8uc1_img.rows; y++)
        for (x = 0; x < src_8uc1_img.cols; x++)
        {
            // Check mask
            empty = get_safe_value(src_8uc1_img, x, y) == 0;
            if (empty)
                continue;

            for (dy = 0; dy < mask_height; dy++)
            {
                sy = dy + y;
                for (dx = 0; dx < mask_height; dx++)
                {
                    if (dx == x && dy == y)
                        continue;

                    sx = dx + x;
                    empty = get_safe_value(src_8uc1_img, sx, sy) == 0;
                    if (empty)
                        break;
                }
                if (empty)
                    break;
            }

            // Empty center
            if (empty)
                if (is_safe(src_8uc1_img, x, y))
                    result.at<unsigned char>(y, x) = 0;
        }

    result.copyTo(src_8uc1_img);
}

void dilate_and_erode_edgemap( cv::Mat & edgemap_8uc1_img ) {
    dilatate_image(edgemap_8uc1_img, 3, 3);
    erode_image(edgemap_8uc1_img, 3, 3);
}

void create_images(cv::Mat& heightmap_8uc1_img, cv::Mat& heightmap_show_8uc3_img, cv::Mat& edgemap_8uc1_img, LidarRanges& lidar_ranges)
{
    // fill the image with data from lidar scanning
    fill_image(lidar_ranges.filename_, heightmap_8uc1_img, lidar_ranges.min_x_, lidar_ranges.max_x_, lidar_ranges.min_y_, lidar_ranges.max_y_, lidar_ranges.min_z_, lidar_ranges.max_z_, lidar_ranges.type_);
    cv::cvtColor(heightmap_8uc1_img, heightmap_show_8uc3_img, CV_GRAY2RGB);

    // create edge map from the height image
    make_edges(heightmap_8uc1_img, edgemap_8uc1_img);

    // binarize image, so we can easily process it in the next step
    binarize_image(edgemap_8uc1_img);

    // implement image dilatation and erosion
    dilate_and_erode_edgemap(edgemap_8uc1_img);
}

/**
 * Mouse clicking callback.
 */
void mouse_probe_handler(int event, int x, int y, int flags, void* param) {
    MouseProbe* probe = (MouseProbe*)param;

    switch (event) {

    case cv::EVENT_LBUTTONDOWN:
        printf("Clicked LEFT at: [ %d, %d ]\n", x, y);
        flood_fill(probe->edgemap_8uc1_img_, probe->heightmap_show_8uc3_img_, x, y);
        break;

    case cv::EVENT_RBUTTONDOWN:
        printf("Clicked RIGHT at: [ %d, %d ]\n", x, y);
        probe->lidar_ranges_.offset_ = (probe->lidar_ranges_.offset_ + 3) % 21;
        probe->lidar_ranges_.type_ = probe->lidar_ranges_.base_type_ >> probe->lidar_ranges_.offset_;
        create_images(probe->heightmap_8uc1_img_, probe->heightmap_show_8uc3_img_, probe->edgemap_8uc1_img_, probe->lidar_ranges_);
        printf("Showing types: %x\n", (probe->lidar_ranges_.type_ & 7));
        break;
    }
}


void create_windows(const int width, const int height) {
    cv::namedWindow(STEP1_WIN_NAME, 0);
    cv::namedWindow(STEP2_WIN_NAME, 0);

    cv::resizeWindow(STEP1_WIN_NAME, width * ZOOM, height * ZOOM);
    cv::resizeWindow(STEP2_WIN_NAME, width * ZOOM, height * ZOOM);

} // create_windows

void transform_sjtsk_to_wgs84(LidarRanges ranges)
{
    projPJ wgs84, sjtsk;
    double min_x, min_y, min_z, max_x, max_y, max_z;
    int p;

    if (!(wgs84 = pj_init_plus("+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs")))
        return exit(1);
    if (!(sjtsk = pj_init_plus("+proj=krovak +ellps=bessel +towgs84=570.8,85.7,462.8,4.998,1.587,5.261,3.56")))
        return exit(1);
    /*if (!(wgs84 = pj_init_plus("+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs")))
        return exit(1);
    if (!(sjtsk = pj_init_plus("+proj=krovak +lat_0=49.5 +lon_0=24.83333333333333 +alpha=30.28813972222222 +k=0.9999 +x_0=0 +y_0=0 +ellps=bessel +pm=greenwich +units=m +no_defs +towgs84=570.8,85.7,462.8,4.998,1.587,5.261,3.56")))
        return exit(1);*/

    min_x = (double)ranges.min_x_;
    min_y = (double)ranges.min_y_;
    min_z = (double)ranges.min_z_;
    max_x = (double)ranges.max_x_;
    max_y = (double)ranges.max_y_;
    max_z = (double)ranges.max_z_;
    p = pj_transform(sjtsk, wgs84, 1, 1, &min_x, &min_y, &min_z);
    p = pj_transform(sjtsk, wgs84, 1, 1, &max_x, &max_y, &max_z);

    pj_free(wgs84);
    pj_free(sjtsk);

    printf("min x: %f, max x: %f\n", min_x * RAD_TO_DEG, max_x * RAD_TO_DEG);
    printf("min y: %f, max y: %f\n", min_y * RAD_TO_DEG, max_y * RAD_TO_DEG);
    printf("min z: %f, max z: %f\n", min_z * RAD_TO_DEG, max_z * RAD_TO_DEG);
}

void process_lidar( const char *txt_filename, const char *bin_filename, const char *img_filename ) {
    float min_x, max_x, min_y, max_y, min_z, max_z;
    float delta_x, delta_y, delta_z;
    MouseProbe *mouse_probe;

    cv::Mat heightmap_8uc1_img_r;
    cv::Mat heightmap_8uc1_img;      // image of source of lidar data
    cv::Mat heightmap_show_8uc3_img; // image to detected areas
    cv::Mat edgemap_8uc1_img;        // image for edges

    get_min_max( bin_filename, &min_x, &max_x, &min_y, &max_y, &min_z, &max_z );

    printf( "min x: %f, max x: %f\n", min_x, max_x );
    printf( "min y: %f, max y: %f\n", min_y, max_y );
    printf( "min z: %f, max z: %f\n", min_z, max_z );

    delta_x = max_x - min_x;
    delta_y = max_y - min_y;
    delta_z = max_z - min_z;

    printf( "delta x: %f\n", delta_x );
    printf( "delta y: %f\n", delta_y );
    printf( "delta z: %f\n", delta_z );

    LidarRanges lidar_ranges(
        bin_filename,
        min_x, max_x, min_y, max_y, min_z, max_z,
        delta_x, delta_y, delta_z
    );

    transform_sjtsk_to_wgs84(lidar_ranges);

    // create images according to data from the source file
    heightmap_8uc1_img = cv::Mat( cvSize( cvRound( delta_x + 0.5f ), cvRound( delta_y + 0.5f ) ), CV_8UC1 );
    heightmap_show_8uc3_img = cv::Mat( cvSize( cvRound( delta_x + 0.5f ), cvRound( delta_y + 0.5f ) ), CV_8UC3 );
    edgemap_8uc1_img = cv::Mat( cvSize( cvRound( delta_x + 0.5f ), cvRound( delta_y + 0.5f ) ), CV_8UC1 );

    create_windows( heightmap_8uc1_img.cols, heightmap_8uc1_img.rows );
    mouse_probe = new MouseProbe( heightmap_8uc1_img, heightmap_show_8uc3_img, edgemap_8uc1_img, lidar_ranges );

    cv::setMouseCallback( STEP1_WIN_NAME, mouse_probe_handler, mouse_probe );
    cv::setMouseCallback( STEP2_WIN_NAME, mouse_probe_handler, mouse_probe );

    printf( "Image w=%d, h=%d\n", heightmap_8uc1_img.cols, heightmap_8uc1_img.rows );
    
    create_images(heightmap_8uc1_img, heightmap_show_8uc3_img, edgemap_8uc1_img, lidar_ranges);

    cv::flip(heightmap_8uc1_img, heightmap_8uc1_img_r, 0);
    cv::imwrite( img_filename, heightmap_8uc1_img_r );

    // wait here for user input using (mouse clicking)
    while ( 1 ) {
        cv::imshow( STEP1_WIN_NAME, heightmap_show_8uc3_img );
        cv::imshow( STEP2_WIN_NAME, edgemap_8uc1_img );
        int key = cv::waitKey(10);
        if ( key == 'q' ) {
            break;
        }
    }
}


int main( int argc, char *argv[] ) {
    /*char *txt_file, *bin_file, *img_file;

    printf("argc: %d\n", argc );
    
    if ( argc < 4 ) {
        printf( "Not enough command line parameters.\n" );
        exit( 1 );
    }

    txt_file = argv[ 1 ];
    bin_file = argv[ 2 ];
    img_file = argv[ 3 ];

    process_lidar( txt_file, bin_file, img_file );*/

    process_lidar("pt000023.txt", "pt000023.bin", "pt000023.png");

    return 0;
}
