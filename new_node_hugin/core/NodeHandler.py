import flatbuffers
from new_node_hugin.link_dev import EgoMotion
from draive_link2 import NodeResources, NodeDiscovery
from draive_link2 import InputPin, OutputPin, Metadata, FlatBuffer
from draive_link2 import SignalHandler, LINK2_SIGNAL_INTERRUPT

class EgoMotionNodeHandler:
    """
    EgoMotionNodeHandler organizes the node operations including:
      - Loading node resources and initializing node discovery.
      - Configuring input and output pins.
      - Setting up signal handling.
      - Preparing and publishing ego motion data via flatbuffers.
    """

    def prepare_node(self, args):
        """
        Load the node resources and announce the node is online.
        
        :param args: Additional arguments required to load the node resources.
        :return: A tuple (node_resources, node_discovery)
        """
        # Load the node resources for our node
        node_resources = NodeResources("l2spec:/link_dev/new-node-hugin", args)
        
        # Announce that our node is "online"
        node_discovery = NodeDiscovery(node_resources)
        
        return node_resources, node_discovery

    def initialize_input_pin(self, node_discovery, node_resources, pull_timeout):
        """
        Initialize the input pin with a specified pull timeout.
        
        :param node_discovery: The node discovery object.
        :param node_resources: The node resources object.
        :param pull_timeout: Timeout value for pulling input data.
        :return: The initialized input pin.
        """
        input_pin = InputPin(node_discovery, node_resources, "hugin-radar-data-input")
        input_pin.setPullTimeout(pull_timeout)
        
        return input_pin

    def initialize_egomotion_output_pin(self, node_discovery, node_resources):
        """
        Initialize the output pin for ego motion data.
        
        :param node_discovery: The node discovery object.
        :param node_resources: The node resources object.
        :return: The initialized output pin.
        """
        output_pin = OutputPin(node_discovery, node_resources, "ego-motion-output")
        
        return output_pin

    def handle_signal(self, receive_signal_timeout):
        """
        Create and configure a signal handler with a receive timeout.
        
        :param receive_signal_timeout: Timeout for receiving signals.
        :return: The configured signal handler.
        """
        signal_handler = SignalHandler()
        signal_handler.setReceiveSignalTimeout(receive_signal_timeout)
        
        return signal_handler

    def prepare_egomotion_data_output(self, measured_speed, estimated_speed, measured_acc, estimated_acc, yaw, pitch, roll, heading):
        """
        Prepare a flatbuffer containing ego motion data.
        
        :param measured_speed: Measured speed value.
        :param estimated_speed: Estimated speed value.
        :param measured_acc: Measured acceleration.
        :param estimated_acc: Estimated acceleration.
        :param yaw: Yaw rate.
        :param pitch: Pitch rate.
        :param roll: Roll rate.
        :param heading: Heading angle.
        :return: A FlatBuffer containing the ego motion data.
        """
        fbb = flatbuffers.Builder(0)
        
        # Start building the EgoMotion table.
        EgoMotion.EgoMotionStart(fbb)
        
        # Add values to the table.
        EgoMotion.EgoMotionAddMeasuredSpeed(fbb, measured_speed)
        EgoMotion.EgoMotionAddEstimatedSpeed(fbb, estimated_speed)
        EgoMotion.EgoMotionAddMeasuredAcceleration(fbb, measured_acc)
        EgoMotion.EgoMotionAddEstimatedAcceleration(fbb, estimated_acc)
        EgoMotion.EgoMotionAddYawRate(fbb, yaw)
        EgoMotion.EgoMotionAddPitchRate(fbb, pitch)
        EgoMotion.EgoMotionAddRollRate(fbb, roll)
        EgoMotion.EgoMotionAddHeading(fbb, heading)
        
        # End the table and finish the buffer.
        egomotion_data = EgoMotion.EgoMotionEnd(fbb)
        fbb.Finish(egomotion_data)
        
        return FlatBuffer(fbb.Output())

    def publish_egomotion_to_output(self, buffer, measured_speed, estimated_speed, measured_acc, estimated_acc, yaw, pitch, roll, heading, meta, output_pin):
        """
        Publish the ego motion data to the output pin.
        
        This method prepares the ego motion data and then pushes it
        along with associated metadata to the output pin.
        
        :param buffer: Flatbuffer buffer (this parameter will be replaced).
        :param measured_speed: Measured speed value.
        :param estimated_speed: Estimated speed value.
        :param measured_acc: Measured acceleration.
        :param estimated_acc: Estimated acceleration.
        :param yaw: Yaw rate.
        :param pitch: Pitch rate.
        :param roll: Roll rate.
        :param heading: Heading angle.
        :param meta: Metadata (this parameter will be reinitialized).
        :param output_pin: The output pin used for publishing the data.
        :return: None
        """
        # Prepare the flatbuffer with the ego motion data.
        buffer = self.prepare_egomotion_data_output(
            measured_speed, estimated_speed, measured_acc, estimated_acc,
            yaw, pitch, roll, heading
        )
        # Create and configure metadata.
        meta = Metadata()
        meta.setSupply("ego-motion")
        output_pin.pushFlatBuffer(buffer, meta)
