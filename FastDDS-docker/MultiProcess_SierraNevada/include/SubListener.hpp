// Templated subscriber listener
// Inherit from DataReaderListener to override some methods
// to get statistics when we get a message

using namespace eprosima::fastdds::dds;

template<typename MsgType>
class SubListener : public DataReaderListener
{
public:
    SubListener(const std::string & name) : name_(name) {}

    ~SubListener() override {}

    // Print something when subscription have matched.
    void on_subscription_matched(
            DataReader*,
            const SubscriptionMatchedStatus& info) override
    {
        if (info.current_count_change == 1) {
            std::cout << "Subscriber " << name_ << " matched." << std::endl;
        }
        else if (info.current_count_change == -1) {
            std::cout << "Subscriber " << name_ << " unmatched." << std::endl;
        } else {
            std::cout << info.current_count_change
                      << " is not a valid value for SubscriptionMatchedStatus current count change"
                      << std::endl;
        }
    }

    // When we get a message: Get stats
    void on_data_available(DataReader* reader) override
    {
        SampleInfo info;
        if (reader->take_next_sample(&msg_, &info) == ReturnCode_t::RETCODE_OK) {
            if (info.instance_state == ALIVE) {
                get_stats();
            }
        }
    }

    template <typename DataT>
    typename std::enable_if<
      (!std::is_same<DataT, std::vector<uint8_t>>::value), size_t>::type
    get_msg_size(DataT & data)
    {
        return msg_.header().size();
    }

    template <typename DataT>
    typename std::enable_if<
      (std::is_same<DataT, std::vector<uint8_t>>::value), size_t>::type
    get_msg_size(DataT & data)
    {
        return data.size();
    }

    void get_stats()
    {
        bool late = false;
        bool too_late = false;

        received_messages_++;

        last_latency_ = get_latency(msg_);
        total_latency_ += last_latency_;

        // I should get this only for the 1st message
        frequency_ = msg_.header().frequency();
        size_ = get_msg_size(msg_.data());

        // Check if we received the correct message. The assumption here is
        // that the messages arrive in chronological order
        if (msg_.header().tracking_number() == tracking_number_count_) {
            tracking_number_count_++;
        } else {
            // We missed some mesages...
            long unsigned int n_lost = msg_.header().tracking_number() - tracking_number_count_;
            lost_messages_ += n_lost;
            tracking_number_count_ = msg_.header().tracking_number() + 1;
        }

        // Check if the message latency qualifies the message as a lost or late message.
        const int  period_us = 1000000 / frequency_;
        // Late message: 20% period - Too late: 100% period
        const unsigned int latency_late_threshold_us = 20 * period_us / 100;
        const unsigned int latency_too_late_threshold_us = period_us;

        too_late = static_cast<long>(last_latency_) > latency_too_late_threshold_us;
        late = static_cast<long>(last_latency_) > latency_late_threshold_us && !too_late;

        if(late) {
            late_messages_++;
        }

        if(too_late) {
            toolate_messages__++;
        }

        //Debug: Print instantaneous latency and tracking number
        // std::cout << "[SUB " << name_ << "] Tracking_number: "
        //           << msg_.header().tracking_number()
        //           << " Latency: " << get_latency(msg_) << " us."
        //           << std::endl;
    }

    long get_latency(MsgType& msg)
    {
        // Get time now
        auto now = std::chrono::high_resolution_clock::now();

        // Get nanoseconds since epoch and compute the diff
        auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
        long now_ns_epoch = now_ns.time_since_epoch().count();

        long msg_timestamp = (static_cast<long>(msg.header().sec()) * 1000000000) + msg.header().nanosec();

        // Return latency in us
        return (now_ns_epoch - msg_timestamp) / 1000;
    }

    void print_stats()
    {
        double average_latency = std::round(total_latency_ / received_messages_);
        double lost_messages_percentage = (double)lost_messages_ / (received_messages_ + lost_messages_) * 100;
        double total_late_percentage = (double)late_messages_ / received_messages_ * 100;
        double total_too_late_percentage = (double)toolate_messages__ / received_messages_ * 100;

        std::cout << "\nSub: " << name_ << ", size: " << size_ << ", Avg latency: " << average_latency
                  << " us, Received msgs: " << received_messages_ << std::endl;
        std::cout << "Late: " << total_late_percentage << "%, Too late: " <<  total_too_late_percentage
                  << "%, Lost: " << lost_messages_percentage << "%" << std::endl;
    }

    MsgType msg_;
    const std::string name_;
    unsigned long int last_latency_ = 0;
    unsigned long int total_latency_ = 0;
    unsigned long int lost_messages_ = 0;
    unsigned long int received_messages_ = 0;
    unsigned long int late_messages_ = 0;
    unsigned long int toolate_messages__ = 0;
    uint32_t tracking_number_count_ = 0;
    size_t size_ = 0;
    float frequency_ = 0;
};
