/*
 * Description:  Test painting with Pen on a rotated box.
 *               The same texture is displayed on each face of the box.
 */

#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/pen.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

WbDeviceTag camera;
const int width = 64;
const int height = 64;
const int size = 4096;

typedef struct {
  int r;
  int g;
  int b;
} Color;

bool isGray(Color color) {
  return (color.r == color.g) && (color.g == color.b);
}

int computeNumColorPixels() {
  int x, y;
  int numColorPixels = 0;
  Color color = {0, 0, 0};

  const unsigned char *image = wb_camera_get_image(camera);
  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x++) {
      color.r = wb_camera_image_get_red(image, width, x, y);
      color.g = wb_camera_image_get_green(image, width, x, y);
      color.b = wb_camera_image_get_blue(image, width, x, y);
      if (!isGray(color)) {
        numColorPixels++;
        // printf("Pixel(%d, %d): r=%d, g=%d, b=%d\n", x, y, wb_camera_image_get_red(image, width, x, y),
        //        wb_camera_image_get_green(image, width, x, y), wb_camera_image_get_blue(image, width, x, y));
      }
    }
  }

  return numColorPixels;
}

Color getPixelColor(int x, int y) {
  Color color = {0, 0, 0};

  const unsigned char *image = wb_camera_get_image(camera);
  color.r = wb_camera_image_get_red(image, width, x, y);
  color.g = wb_camera_image_get_green(image, width, x, y);
  color.b = wb_camera_image_get_blue(image, width, x, y);
  return color;
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  WbDeviceTag pen, motor, position_sensor;
  Color color;
  double pos;
  int oldValue, numColorPixels = -1;

  pen = wb_robot_get_device("pen");
  wb_pen_set_ink_color(pen, 0xFA8A0A, 1.0);
  wb_pen_write(pen, true);

  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  motor = wb_robot_get_device("linear motor");

  position_sensor = wb_robot_get_device("position sensor");
  wb_position_sensor_enable(position_sensor, TIME_STEP);

  wb_robot_step(TIME_STEP);

  wb_pen_write(pen, false);

  // 1) Test initial painted area location
  // ts_send_error_and_exit
  numColorPixels = computeNumColorPixels();
  ts_assert_boolean_equal(numColorPixels <= 20,
                          "The number of pixels painted after the first step should be lower than 20 not %d", numColorPixels);

  color = getPixelColor(13, 56);
  ts_assert_color_in_delta(
    color.r, color.g, color.b, 185, 152, 60, 15,
    "Pixel (13, 56) should be painted with color [r=%d, g=%d, b=%d] not [r=%d, g=%d, b=%d] after first paint call", 185, 152,
    60, color.r, color.g, color.b);

  wb_robot_step(TIME_STEP);

  // move pen
  pos = wb_position_sensor_get_value(position_sensor);
  pos = pos + 0.02;
  wb_motor_set_position(motor, pos);

  wb_robot_step(TIME_STEP);
  // print
  wb_pen_write(pen, true);

  wb_robot_step(TIME_STEP);
  wb_pen_write(pen, false);

  // 2) Test second painted area location
  oldValue = numColorPixels;
  numColorPixels = computeNumColorPixels();

  ts_assert_boolean_equal(
    oldValue <= numColorPixels && numColorPixels <= 40,
    "The number of pixels painted after the second step should be greater than before (%d) and lower than 40 not %d", oldValue,
    numColorPixels);

  color = getPixelColor(28, 39);
  ts_assert_color_in_delta(
    color.r, color.g, color.b, 185, 151, 50, 15,
    "Pixel (28, 39) should be painted with color [r=%d, g=%d, b=%d] not [r=%d, g=%d, b=%d] after second paint call", 185, 151,
    50, color.r, color.g, color.b);

  wb_robot_step(TIME_STEP);

  // move pen
  pos = wb_position_sensor_get_value(position_sensor);
  pos = pos + 0.02;
  wb_motor_set_position(motor, pos);

  wb_robot_step(TIME_STEP);
  // print
  wb_pen_write(pen, true);

  wb_robot_step(TIME_STEP);
  wb_pen_write(pen, false);

  // 3) Test third painted area location
  oldValue = numColorPixels;
  numColorPixels = computeNumColorPixels();

  ts_assert_boolean_equal(
    oldValue <= numColorPixels && numColorPixels <= 60,
    "The number of pixels painted after the third step should be greater than before (%d) and lower than 60 not %d", oldValue,
    numColorPixels);

  color = getPixelColor(34, 25);
  ts_assert_color_in_delta(
    color.r, color.g, color.b, 185, 151, 51, 15,
    "Pixel (34, 25) should be painted with color [r=%d, g=%d, b=%d] not [r=%d, g=%d, b=%d] after third paint call", 185, 151,
    51, color.r, color.g, color.b);

  ts_send_success();
  return EXIT_SUCCESS;
}
