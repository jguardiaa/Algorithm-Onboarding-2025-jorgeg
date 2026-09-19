/*
 *  READ ALL COMMENTS IN THIS FILE TO UNDERSTAND THE CODE THAT HAS BEEN WRITTEN FOR YOU
 *
 *  This file will create a subscriber node that reads images from the topic where camera_publisher_node publishes
 *  images. It will contain the functionality necessary for locating armor plates within a given image.
 *
 *  Since this file implements a ROS node like camera_publisher_node does, you may want to refer to that file to get
 *  ideas for things you need to consider while writing your code.
 */

#include "../include/armor_detector/armor_detector_node.hpp"

/*
 *  This is the constructor for our subscriber node. It initializes the node inherited from the base class and creates
 *  the subscription to the topic with messages from camera_publisher_node.
 */
ArmorDetectorNode::ArmorDetectorNode() : Node("armor_detector_node"), frame_count(0)
{
    // Subscribe to the camera publisher topic
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&ArmorDetectorNode::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ArmorDetectorNode subscribed to topic");
}

/*
 *  This is an image callback method. It fetches messages (which are images in this case) from the topic this node
 *  subscribes to. The method will also run your armor detection algorithm on the image and show the result.
 *
 *  Callback methods are how we actually read data from a topic. Notice the parameter type and compare it to that of
 *  image_sub_. Our subscription here fetches images, and this method is how you actually operate on that image. Even
 *  though your image processing logic is in a different method, this callback uses those methods as helpers. Make sure
 *  that you understand how callbacks function in a ROS node architecture. If you completed the constructor correctly,
 *  you should know how a callback connects to a subscription in code.
 */
void ArmorDetectorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Images are represented by cv::Mat objects    std::bind(&image_callback, this, std::placeholders::_1)    std::bind(&image_callback, this, std::placeholders::_1)    std::bind(&image_callback, this, std::placeholders::_1)    std::bind(&image_callback, this, std::placeholders::_1)    std::bind(&image_callback, this, std::placeholders::_1). This will be useful when you write image processing logic.
    cv::Mat frame;
    
    // Read the image from the topic into our frame with the proper color space (BGR8)
    try {
        frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::vector<cv::RotatedRect> armors = search(frame, lowerHSV, upperHSV, lowerHSV2, upperHSV2);
    frame_count++;

    if (armors.size() == 2)
    {
        auto p0 = rect_to_point(armors[0]);
        auto p1 = rect_to_point(armors[1]);
        std::cout << frame_count << "," << p0[0] << "," << p0[1] << "," << p1[0] << "," << p1[1] << std::endl;

        draw_rotated_rect(frame, armors[0]);
        draw_rotated_rect(frame, armors[1]);
    }
    else
    {
        std::cout << frame_count << "," << "no armor found" << std::endl;
    }

    // Reduce the computational load and just show every 5th image
    if (frame_count % 5 == 0)
    {
        show_frame(frame);
    }
}

/*
 *  This method displays a frame to your screen.
 */
void ArmorDetectorNode::show_frame(cv::Mat &frame)
{
    std::vector<uchar> buf;
    cv::resize(frame, frame, cv::Size(640, 480));
    cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 20});
    cv::imshow("Detection Frame", cv::imdecode(buf, cv::IMREAD_COLOR));
    if (cv::waitKey(1) == 27)
    {
        cv::destroyAllWindows();
        rclcpp::shutdown();
    }
}

/*
 *  The main method activates this subscriber node.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorDetectorNode>());
    rclcpp::shutdown();
    return 0;
}

/*
 *  This method will search a frame for armor plates and return the RotatedRect objects that correspond to the two light
 *  bars that exist on an armor plate.
 */
std::vector<cv::RotatedRect> ArmorDetectorNode::search(cv::Mat& frame, cv::Scalar lowerHSV, cv::Scalar upperHSV, cv::Scalar lowerHSV2, cv::Scalar upperHSV2) {
    cv::Mat framesmth;
    cv::bilateralFilter(frame, framesmth, 9, 75, 75);
    cv::Mat framehsv;
    cv::cvtColor(framesmth, framehsv, cv::COLOR_BGR2HSV);

    

    //created a new framehsv converted from bgr to hsv and applied bilateral filter for smoothing 


    cv::Mat mask1;
    cv::Mat mask2;
    cv::Mat cmbmask;

    cv::inRange(framehsv, lowerHSV, upperHSV, mask1);
    cv::inRange(framehsv, lowerHSV2, upperHSV2, mask2);
    cv::bitwise_or(mask1, mask2, cmbmask);

    cv::Mat cnyresult;
    cv::Canny(cmbmask, cnyresult, 100, 300);
    std::vector<std::vector<cv::Point>> contrs;
    cv::findContours(cnyresult, contrs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < contrs.size(); i++) {
        if (cv::contourArea(contrs[i]) < 100) {
            continue;
        }
    }




    //created two masks for the two color ranges and combined them into one mask using bitwise_or like the document siad


    

    // TODO: Complete the rest of the method. The onboarding instructions document will be very helpful.

    // 1) Image Preprocessing

    // 2) Color segmentation

    // 2.5) Edge Detection
    
    // 3) Contour Detection

    // 4) Contour Filtering

    return {}; // Default return value, no armor found
}

/*
 *  This method draws a rotated rectangle onto a frame. This is used to display the results of your algorithm when you
 *  run the node.
 */
void ArmorDetectorNode::draw_rotated_rect(cv::Mat &frame, cv::RotatedRect &rect)
{
    cv::Point2f vertices[4];
    rect.points(vertices);
    for (int i = 0; i < 4; i++)
    {
        cv::line(frame, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
    }
}

/*
 *  This method determines whether a RotatedRect object can represent a light bar based on the constants defined in the
 *  header file. It checks dimensions, angles, and ratios against our configured thresholds to do so.
 */
bool ArmorDetectorNode::is_light_bar(cv::RotatedRect &rect)
{
    //constants that i need for ts stuff idk bro:
/*  LIGHT_BAR_ANGLE_LIMIT 30.0
    LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT 2.0
    LIGHT_BAR_WIDTH_LOWER_LIMIT 2.0
    LIGHT_BAR_HEIGHT_LOWER_LIMIT 5.0 */

    if (rect.size.width < LIGHT_BAR_WIDTH_LOWER_LIMIT || rect.size.height < LIGHT_BAR_HEIGHT_LOWER_LIMIT) {
        return false; // Width or height is below the lower limit
    }
    if (rect.angle < -LIGHT_BAR_ANGLE_LIMIT || rect.angle > LIGHT_BAR_ANGLE_LIMIT) {
        return false; // Angle is outside the valid range
    }
    if (rect.size.height / rect.size.width < LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT) {
        return false; // Aspect ratio is below the lower limit
    }
    if (rect.size.height / rect.size.width > 1.0 / LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT) {
        return false; // Aspect ratio is above the upper limit
    }
    else {
        return true; // All checks passed, it's a valid light bar
    }
    


   // TODO: Use the LIGHT_BAR constants defined in the header file to complete this method.
    // You may want to read the OpenCV documentation for RotatedRect

    // Verify that the light bar width is valid

    // Verify that the light bar height is valid

    // Verify that the light bar angle is valid
    // You will want to compare against both the limit and its supplement; think about the unit circle

    // Verify that the light bar aspect ratio is valid
    // Aspect ratio refers to height / width, not width / height
}

/*
 *  This method determines whether a pair of light bars (RotatedRect objects) can represent an armor plate based on the
 *  constants defined in the header file. It checks dimensions, angles, and ratios against our configured thresholds to
 *  do so.
 */
bool ArmorDetectorNode::is_armor(cv::RotatedRect &left_rect, cv::RotatedRect &right_rect)
{

/*     #define ARMOR_ANGLE_DIFF_LIMIT 5.0
    #define ARMOR_LIGHT_BAR_ASPECT_RATIO_RATIO_LIMIT 5.0
    #define ARMOR_Y_DIFF_LIMIT 1.5
    #define ARMOR_HEIGHT_RATIO_LIMIT 1.5
    #define ARMOR_ASPECT_RATIO_LIMIT 2.5 */

    if (abs(left_rect.angle - right_rect.angle) > ARMOR_ANGLE_DIFF_LIMIT) {
        return false; // Angle difference exceeds limit
    }
    if (left_rect.size.height / left_rect.size.width > ARMOR_LIGHT_BAR_ASPECT_RATIO_RATIO_LIMIT || right_rect.size.height / right_rect.size.width > ARMOR_LIGHT_BAR_ASPECT_RATIO_RATIO_LIMIT) {
        return false; // Aspect ratio exceeds limit
    }
    if (abs(left_rect.center.y - right_rect.center.y) / ((left_rect.size.height + right_rect.size.height) / 2) > ARMOR_Y_DIFF_LIMIT) {
        return false; 
    }
    if (left_rect.size.height / right_rect.size.height > ARMOR_HEIGHT_RATIO_LIMIT || right_rect.size.height / left_rect.size.height > ARMOR_HEIGHT_RATIO_LIMIT) {
        return false; // Height ratio exceeds limit
    }
    if ((left_rect.size.width / left_rect.size.height) > ARMOR_ASPECT_RATIO_LIMIT || (right_rect.size.width / right_rect.size.height) > ARMOR_ASPECT_RATIO_LIMIT) {
        return false; // Armor aspect ratio exceeds limit
    }
    else {
        return true; // All checks passed, it's a valid armor plate
    }
    // TODO: Use the ARMOR constants defined in the header file to complete this method.

    // Verify that the light bars are roughly parallel by checking that their difference does not exceed the threshold
    // Again, you will want to compare against both the limit and its supplement

    // Verify that the ratio between the light bar aspect ratios (that's a mouthful) is within the threshold
    // You will want to compare both left / right and right / left against the threshold

    // Verify that the light bars are at roughly the same elevation (as in their y difference is within the threshold)
    // The way the constant was determined assumes that you normalize this difference using the average light bar height
    // What that means is that the expression you should be checking is abs(y_left - y_right) / avg_height

    // Verify that the ratio between light bar heights is within the threshold
    // Again, you will want to compare both left / right and right / left

    // Verify that the armor aspect ratio is within the threshold
    // For some goofy reason, the constant for this step requires that you calculate aspect ratio as width / height
    // There are multiple ways to define armor plate "height" and "width." Hopefully your idea is effective!
}

/*
 *  This method represents a RotatedRect object as a point and returns it. It exists for debugging output while the node
 *  is being run.
 */
std::vector<cv::Point2f> ArmorDetectorNode::rect_to_point(cv::RotatedRect &rect)
{
    float rad = rect.angle < 90 ? rect.angle * M_PI / 180.f : (rect.angle - 180) * M_PI / 180.f;
    float x_offset = rect.size.height * std::sin(rad) / 2.f;
    float y_offset = rect.size.height * std::cos(rad) / 2.f;

    std::vector<cv::Point2f> points;
    points = std::vector<cv::Point2f>();
    points.push_back(cv::Point2f(int(rect.center.x + x_offset), int(rect.center.y - y_offset)));
    points.push_back(cv::Point2f(int(rect.center.x - x_offset), int(rect.center.y + y_offset)));
    return points;
}
