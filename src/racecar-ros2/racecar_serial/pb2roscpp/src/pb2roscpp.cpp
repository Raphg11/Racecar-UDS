#include <chrono>
#include <thread>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <stdio.h>
#include <bitset>
#include <cstring>

#include "pb_encode.h"
#include "pb_decode.h"
#include "floatarray.pb.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "serial/serial.h"

#define MAX_MSG_LEN 200
#define MAX_NUMBERS 19
using std::placeholders::_1;

std::string port = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0";
long int baud = 115200;

class ArduinoCommunicationNode : public rclcpp::Node
{
  public:
    // Constructor
    ArduinoCommunicationNode() : Node("pb2ros") // is a Node
    {
        // Port configurable via paramètre ROS (défaut : chemin stable by-id de l'Arduino CH340)
        port = this->declare_parameter<std::string>("serial_port", port);

        RCLCPP_INFO(this->get_logger(), "Connecting to Arduino on port: %s", port.c_str());

        my_serialp = new serial::Serial(
            port, baud, serial::Timeout::simpleTimeout(1000)); // start serial com with arduino
        std::this_thread::sleep_for(std::chrono::seconds(2));  // wait for 2 seconds to let it
                                                               // connect
        my_serialp->flushInput(); // flush the input buffer just to start on known ground

        // Is the Serial port open?
        if (!my_serialp->isOpen()) {
            std::cerr << "Failed to open serial port" << std::endl;
            return;
        }

        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "prop_sensors", 1); // create publisher object
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "prop_cmd", 1,
            std::bind(&ArduinoCommunicationNode::controlCallback, this,
                      _1)); // create subscriber object

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&ArduinoCommunicationNode::readMessage,
                      this)); // create the reading/publishing loop for the topic /prop_sensors
    }

    ~ArduinoCommunicationNode()
    {
        delete my_serialp; // deleting the dynamicaly allocated serial object
    }

  private:
    // Member object
    rclcpp::TimerBase::SharedPtr timer_; // readmessage loop timer
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>> publisher_;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> subscription_;

    serial::Serial
        *my_serialp; // Serial pointer to initialize the serial communication in the constructor

    // Fonctions
    void controlCallback(const geometry_msgs::msg::Twist msg)
    {
        // declare variables
        uint8_t buffer[MAX_MSG_LEN];

        // prepare the actual "variable" array
        FloatList actualData = {};
        FloatList_add_number(&actualData, (float)msg.angular.z);
        FloatList_add_number(&actualData, (float)msg.linear.x);
        FloatList_add_number(&actualData, (float)msg.linear.z);

        // init the message
        FloatArray message = FloatArray_init_zero;

        // put the data in the message
        message.data.arg = &actualData;

        // link the encode function for float arrays to the message object
        message.data.funcs.encode = FloatArray_encode_numbers;

        /* Create a stream that will write to our buffer. */
        pb_ostream_t ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));

        // encode the message
        if (!pb_encode(&ostream, FloatArray_fields, &message)) {
            const char *error = PB_GET_ERROR(&ostream);
            printf("pb_encode error: %s\n", error);
            return;
        }

        // Define vars for stringifing the encoded buffer
        size_t total_bytes_encoded = ostream.bytes_written;
        char toSend[10];
        std::string strmsg;
        std::string begining = "<1|";
        std::string end = ";>";

        // stringify
        for (int j = 0; j < (int)total_bytes_encoded; ++j) {
            sprintf(toSend, "%02X", buffer[j]);
            strmsg += toSend;
        }
        // strmsg+='\0';
        // build the message format
        begining.append(strmsg);
        begining.append(end);

        // send the message

        // std::cout<<(char*)begining.c_str()<<std::endl;
        my_serialp->write((char *)begining.c_str());
    }

    void readMessage() // reading arduino messages callback
    {
        std::string msgbuffer;
        int msgids[MAX_MSG_LEN];
        char *msgs[MAX_MSG_LEN] = {nullptr};
        uint8_t bufferin[MAX_MSG_LEN] = {0};
        // empty array for decoding
        FloatList decodedData = {};
        /* Allocate space for the decoded message. */
        FloatArray message = FloatArray_init_zero;
        message.data.arg = &decodedData;
        message.data.funcs.decode = FloatArray_decode_single_number;

        if (my_serialp->available() > 0) {

            my_serialp->readline(msgbuffer, MAX_MSG_LEN, ">"); // readuntil the end of a message

            // Garde-fou : ignorer les trames incomplètes ou corrompues (bruit au démarrage, reset de l'Arduino...)
            size_t start = msgbuffer.rfind('<');
            if (start == std::string::npos || msgbuffer.empty() || msgbuffer.back() != '>') {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Trame Arduino incomplete ignoree");
                return;
            }
            std::string frame = msgbuffer.substr(start + 1, msgbuffer.size() - start - 2); // contenu entre < et >

            // parse the message between Id and message parts
            int nbMsgs = parseMsg(&frame[0], msgids, msgs);
            if (nbMsgs < 1 || msgs[0] == nullptr) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Trame Arduino invalide ignoree");
                return;
            }

            size_t hexLen = strlen(msgs[0]);
            if (hexLen < 2 || hexLen % 2 != 0 || hexLen / 2 > MAX_MSG_LEN) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Longueur de trame Arduino invalide ignoree");
                return;
            }

            size_t nbBytes = charsToBytes(msgs[0], bufferin); // convert to bytes for decoding

            pb_istream_t istream = pb_istream_from_buffer(
                bufferin,
                nbBytes); // seting up decoding stream with the actual size of the incoming message
            if (!pb_decode(&istream, FloatArray_fields, &message)) // actualy try to decode message
            {
                const char *error = PB_GET_ERROR(&istream);
                printf("pb_decode error: %s\n", error);
                return;
            }

            std_msgs::msg::Float32MultiArray prop_sensors; // instantiate the published message

            for (int i = 0; i < decodedData.numbers_count; i++) {
                prop_sensors.data.push_back(
                    decodedData.numbers[i]); // append the decoded message to the data
            }

            publisher_->publish(prop_sensors); // publish the data
        }
    }

    typedef struct {
        float numbers[MAX_NUMBERS];
        int32_t numbers_count;
    } FloatList;

    static int parseMsg(char inString[], int *msgIds, char *outStrings[])
    {
        // Split all messages
        char *wholeMsgs[MAX_MSG_LEN];
        int ind = 0;
        char *substr = strtok(inString, ";");

        while (substr != NULL && ind < MAX_MSG_LEN) {
            wholeMsgs[ind++] = substr;
            substr = strtok(NULL, ";");
        }

        // Split (id, msg) pair
        for (int i = 0; i < ind; i++) {
            char *id = strtok(wholeMsgs[i], "|");
            char *msg = strtok(NULL, "|");

            msgIds[i] = (id != NULL) ? atoi(id) : -1;
            outStrings[i] = msg; // peut etre NULL si la trame est invalide
        }

        return ind;
    }

    static uint8_t charToHex(char in1,
                             char in2) // Code from PBUtils.cpp (original pb2ros2 codebase) converts
                                       // chars to hex for decoding used by charstobytes
    {
        uint8_t val[2];
        char in[2] = {in1, in2};

        for (int i = 0; i < 2; i++) {
            if (in[i] >= 'A' && in[i] <= 'F')
                val[i] = in[i] - 'A' + 10;
            else if (in[i] >= 'a' && in[i] <= 'f')
                val[i] = in[i] - 'a' + 10;
            else
                val[i] = in[i] - '0';
        }

        return val[0] * 16 + val[1];
    }

    static size_t
    charsToBytes(char *inString,
                 uint8_t *stringValue) // Code from PBUtils.cpp (original pb2ros2 codebase) converts
                                       // chars read from serial port to bytes for decoding
    {
        int len = strlen(inString) / 2;
        for (int i = 0; i < len; ++i)
            stringValue[i] = charToHex(inString[(i * 2)], inString[(i * 2) + 1]);
        return (size_t)len;
    }

    static void FloatList_add_number(
        FloatList *list,
        float number) // custom function to handle adding floats to float arrays in nanopb
    {
        if (list->numbers_count < MAX_NUMBERS) {
            list->numbers[list->numbers_count] = number;
            list->numbers_count++;
        }
    }

    static bool
    FloatArray_decode_single_number(pb_istream_t *istream, const pb_field_t *field,
                                    void **arg) // custom decoding function from stackoverflow
    {
        RCL_UNUSED(field);
        FloatList *dest = (FloatList *)(*arg);

        // decode single number
        float number;
        if (!pb_decode_fixed32(istream, &number)) {
            const char *error = PB_GET_ERROR(istream);
            printf("SimpleMessage_decode_single_number error: %s\n", error);
            return false;
        }

        // add to destination list
        FloatList_add_number(dest, (float)number);
        return true;
    }

    static bool
    FloatArray_encode_numbers(pb_ostream_t *ostream, const pb_field_t *field,
                              void *const *arg) // custom encoding function from stackoverflow
    {
        FloatList *source = (FloatList *)(*arg);

        // encode all numbers
        for (int i = 0; i < source->numbers_count; i++) {
            if (!pb_encode_tag_for_field(ostream, field)) {
                const char *error = PB_GET_ERROR(ostream);
                printf("SimpleMessage_encode_numbers error: %s\n", error);
                return false;
            }

            if (!pb_encode_fixed32(ostream, &source->numbers[i])) {
                const char *error = PB_GET_ERROR(ostream);
                printf("SimpleMessage_encode_numbers error: %s\n", error);
                return false;
            }
        }

        return true;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoCommunicationNode>());
    rclcpp::shutdown();
    return 0;
}