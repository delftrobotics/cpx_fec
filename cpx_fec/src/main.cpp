#include <modbus/client.hpp>
#include <modbus/error.hpp>

#include <cpx_fec_msgs/SetOutput.h>
#include <cpx_fec_msgs/SetOutputs.h>
#include <std_msgs/UInt32.h>

#include <ros/ros.h>

#include <string>
#include <thread>

namespace cpx_fec {

class Node {
	ros::NodeHandle node{"~"};
	ros::Publisher output_publisher;
	ros::ServiceServer set_output_server;
	ros::ServiceServer set_outputs_server;


	modbus::client modbus;
	boost::asio::deadline_timer read_timer;
	boost::asio::deadline_timer reconnect_timer;

	std::string host;
	int port = 502;
	int write_base = 40003;
	int read_base  = 45395;

public:
	Node(boost::asio::io_service & ios) :
		modbus(ios),
		read_timer(ios),
		reconnect_timer(ios)
	{
		modbus.on_io_error = [this] (boost::system::error_code const & error) {
			handleError(error, "communicating with modbus server");
		};

		if (!node.getParam("host", host)) throw std::runtime_error("No ~/host parameter given.");
		node.getParam("port", port);
		node.getParam("write_base", write_base);
		node.getParam("read_base", read_base);

		output_publisher   = node.advertise<std_msgs::UInt32>("outputs", 1, false);
		set_output_server  = node.advertiseService("set_output",  &Node::onSetOuput, this);
		set_outputs_server = node.advertiseService("set_outputs", &Node::onSetOuputs, this);
	}

	void connect() {
		ROS_INFO_STREAM("Connecting to modbus server at " << host << ":" << port << ".");
		modbus.connect(host, "502", std::bind(&Node::onConnect, this, std::placeholders::_1));
	}

	void close() {
		ROS_WARN_STREAM("Closing connection to modbus server.");
		modbus.close();
		read_timer.cancel();
		reconnect_timer.cancel();
	}

protected:
	void startReconnectTimeout() {
		close();
		reconnect_timer.expires_from_now(boost::posix_time::seconds(3));
		reconnect_timer.async_wait([this] (boost::system::error_code const & error) {
			if (handleError(error, "waiting for reconnect timer")) return;
			connect();
		});
	}

	bool handleError(boost::system::error_code const & error, std::string const & message) {
		if (!error) return false;
		if (error == boost::asio::error::operation_aborted) return true;

		ROS_ERROR_STREAM("Error while " << message << ": " << error << ": " << error.message());
		if (error.category() != modbus::modbus_category()) startReconnectTimeout();
		return true;
	}

	void onConnect(boost::system::error_code const & error) {
		if (handleError(error, "connecting to modbus server")) return;

		ROS_INFO_STREAM("Connected to modbus server at " << host << ":" << port << ".");
		onReadTimeout({});
	}

	void onReadTimeout(boost::system::error_code const & error) {
		if (handleError(error, "waiting for read timer")) return;

		auto callback = [this] (modbus::tcp_mbap const &, modbus::response::read_holding_registers const & response, boost::system::error_code const & error) {
			if (handleError(error, "reading valve state from modbus server")) return;
			if (response.values.size() != 2) {
				ROS_ERROR_STREAM("Got wrong number of values from modbus server. Got " << response.values.size() << " , expected 2.");
				return;
			}

			std_msgs::UInt32 message;
			message.data = response.values[0] | response.values[1] << 16;
			output_publisher.publish(message);

			read_timer.expires_from_now(boost::posix_time::milliseconds(50));
			read_timer.async_wait(std::bind(&Node::onReadTimeout, this, std::placeholders::_1));
		};

		modbus.read_holding_registers(0, read_base, 2, callback);
	}

	void writeMasked(std::uint16_t address, std::uint16_t value, std::uint16_t mask) {
		std::uint16_t and_mask =  ~mask;
		std::uint16_t or_mask  =  value & mask;

		ROS_DEBUG_STREAM(
			"Setting register " << address << " to 0x"
			<< std::hex << std::setw(4) << value
			<< " with mask "
			<< std::hex << std::setw(4) << mask
			<< ". Using and_mask 0x"
			<< std::hex << std::setw(4) << and_mask
			<< " with or_mask 0x"
			<< std::hex << std::setw(4) << or_mask
			<< ".";
		);

		auto callback = [this, address] (modbus::tcp_mbap const &, modbus::response::mask_write_register const &, boost::system::error_code const & error) {
			if (handleError(error, "setting register " + std::to_string(address))) return;
		};

		modbus.mask_write_register(0, address, and_mask, or_mask, callback);
	}

	void setOutputs(unsigned int values, unsigned int mask) {
		writeMasked(write_base + 0, values >>  0 & 0xffff, mask >>  0 & 0xffff);
		writeMasked(write_base + 1, values >> 16 & 0xffff, mask >> 16 & 0xffff);
	}

	bool onSetOuput(cpx_fec_msgs::SetOutput::Request & request, cpx_fec_msgs::SetOutput::Response &) {
		if (!modbus.is_connected()) {
			ROS_ERROR_STREAM("Not connected to modbus server. Can not set output.");
			return false;
		}

		modbus.ios().dispatch([this, request]() {
			int offset   = request.index / 16;
			int subindex = request.index % 16;
			writeMasked(write_base + offset, request.value << subindex, 1 << subindex);
		});
		return true;
	}

	bool onSetOuputs(cpx_fec_msgs::SetOutputs::Request & request, cpx_fec_msgs::SetOutputs::Response &) {
		if (!modbus.is_connected()) {
			ROS_ERROR_STREAM("Not connected to modbus server. Can not set outputs.");
			return false;
		}

		modbus.ios().dispatch([this, request]() {
			setOutputs(request.value, request.mask);
		});
		return true;
	}
};

}

int main(int argc, char * * argv) {
	ros::init(argc, argv, ROS_PACKAGE_NAME);
	boost::asio::io_service ios;
	cpx_fec::Node node(ios);
	node.connect();

	std::thread io_thread([&ios] () {
		ios.run();
	});

	ros::spin();
	ios.stop();
	io_thread.join();

	return 0;
}
