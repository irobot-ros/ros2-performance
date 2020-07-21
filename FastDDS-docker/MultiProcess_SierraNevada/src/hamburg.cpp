// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file hamburg.cpp
 * Replicates the performance benchmark of hamburg topology
 *
 */

// Include messages used in hamburg topology
#include "msg/Stamped12Float32PubSubTypes.h"
#include "msg/Stamped3Float32PubSubTypes.h"
#include "msg/Stamped4Float32PubSubTypes.h"
#include "msg/Stamped4Int32PubSubTypes.h"
#include "msg/Stamped9Float32PubSubTypes.h"
#include "msg/StampedInt64PubSubTypes.h"
#include "msg/StampedVectorPubSubTypes.h"

// Publishers includes
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>

// Subscribers includes
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>

// Listener: Where we compute stats from messages
#include "SubListener.hpp"
#include "Macros.hpp"

using namespace eprosima::fastdds::dds;

class Hamburg
{
private:
    // Let's use only one participant and associate all pubs/subs to it
    DomainParticipant* participant_ {nullptr};

    // Declare publishers, writers, subscriber, readers, topics
    // and init by default to nullptr
    DECLARE_SUB(danube)
    DECLARE_SUB(ganges)
    DECLARE_SUB(nile)
    DECLARE_SUB(tigris)
    DECLARE_PUB(parana)

    // Objects to register the topic data type in the DomainParticipant.
    TypeSupport type_stamped_int64_;
    TypeSupport type_stamped4_int32_;
    TypeSupport type_stamped4_float32_;
    TypeSupport type_stamped3_float32_;

    // Create subscriber listeners
    SubListener<StampedInt64>     listener_danube_;
    SubListener<Stamped4Int32>    listener_ganges_;
    SubListener<Stamped4Int32>    listener_nile_;
    SubListener<Stamped4Float32>  listener_tigris_;

    std::atomic_bool run_threads {true};

public:

    Hamburg()
        : type_stamped3_float32_  (new Stamped3Float32PubSubType())
        , type_stamped4_float32_  (new Stamped4Float32PubSubType())
        , type_stamped4_int32_    (new Stamped4Int32PubSubType())
        , type_stamped_int64_     (new StampedInt64PubSubType())
        , listener_danube_("danube")
        , listener_ganges_("ganges")
        , listener_nile_("nile")
        , listener_tigris_("tigris")
    {
    }

    virtual ~Hamburg()
    {
        // Delete data writers, publishers, data readers, subscribers, topics
        DESTRUCTOR_SUB(danube)
        DESTRUCTOR_SUB(ganges)
        DESTRUCTOR_SUB(nile)
        DESTRUCTOR_SUB(tigris)
        DESTRUCTOR_PUB(parana)

        DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }

    bool init()
    {
        // Create participant
        DomainParticipantQos participantQos;
        participantQos.name("Participant");
        participant_ = DomainParticipantFactory::get_instance()->create_participant(0, participantQos);

        // The recently created pointer should not be null
        if (participant_ == nullptr) { return false; }

        // Register the data Types to the participant
        type_stamped4_float32_.register_type(participant_);
        type_stamped3_float32_.register_type(participant_);
        type_stamped4_int32_.register_type(participant_);
        type_stamped_int64_.register_type(participant_);

        // Init the publications/subscription Topics
        danube_   = participant_->create_topic("danube",   "StampedInt64",     TOPIC_QOS_DEFAULT);
        ganges_   = participant_->create_topic("ganges",   "Stamped4Int32",    TOPIC_QOS_DEFAULT);
        nile_     = participant_->create_topic("nile",     "Stamped4Int32",    TOPIC_QOS_DEFAULT);
        parana_   = participant_->create_topic("parana",   "Stamped3Float32",  TOPIC_QOS_DEFAULT);
        tigris_   = participant_->create_topic("tigris",   "Stamped4Float32",  TOPIC_QOS_DEFAULT);

        // Init the Publishers, DataWriters, Subscribers,DataReaders
        INIT_SUB(danube)
        INIT_SUB(ganges)
        INIT_SUB(nile)
        INIT_SUB(tigris)
        INIT_PUB(parana)

        return true;
    }


    int32_t seconds_since_epoch(std::chrono::time_point<std::chrono::high_resolution_clock> now)
    {
        auto now_s = std::chrono::time_point_cast<std::chrono::seconds>(now);
        return static_cast<int32_t>(now_s.time_since_epoch().count());
    }

    uint32_t nanoseconds_diff(std::chrono::time_point<std::chrono::high_resolution_clock> now)
    {
        // Get seconds since epoch
        auto now_s = std::chrono::time_point_cast<std::chrono::seconds>(now);
        long now_s_epoch = now_s.time_since_epoch().count();

        // Get nanoseconds since epoch and compute the diff
        auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
        long now_ns_epoch = now_ns.time_since_epoch().count();

        long now_ns_diff = now_ns_epoch - (now_s_epoch * 1000000000);
        return static_cast<uint32_t>(now_ns_diff);
    }

    // If the data field of the message is a vector, resize it.
    template <typename DataT, typename MsgType>
    typename std::enable_if<(std::is_same<DataT, std::vector<uint8_t>>::value), void>::type
    set_msg_size(MsgType& msg, DataT & data, size_t size)
    {
        data.resize(size);
        msg.header().size() = size;
    }

    // If the data field of the message is not a vector, just set size on header.
    template <typename DataT, typename MsgType>
    typename std::enable_if<(!std::is_same<DataT, std::vector<uint8_t>>::value), void>::type
    set_msg_size(MsgType& msg, DataT & data, size_t size)
    {
        msg.header().size() = size;
    }

    template<typename MsgType>
    void fill_msg(
        MsgType& msg,
        uint32_t tracking_number,
        uint32_t period,
        size_t size,
        const std::string & name)
    {
        msg.header().tracking_number() = tracking_number;
        msg.header().frequency() = 1000.0 / period;

        set_msg_size(msg, msg.data(), size);

        // Get time now
        auto now = std::chrono::high_resolution_clock::now();
        msg.header().sec()     = seconds_since_epoch(now);
        msg.header().nanosec() = nanoseconds_diff(now);

        // Debug
        // std::cout << "[PUB: " << name << "] Tracking_number: " << tracking_number << std::endl;
    }

    // Templated publisher
    template <typename MsgType>
    void publish(
        DataWriter* data_writer,
        uint32_t period,
        size_t size,
        const std::string & name)
    {
        uint32_t tracking_number = 0;

        while(run_threads) {
            MsgType msg;

            // Init messages tracking number, timestamp
            fill_msg(msg, tracking_number, period, size, name);

            tracking_number++;

            //Publish messages
            data_writer->write(&msg);

            std::this_thread::sleep_for(std::chrono::milliseconds(period));
        }

        //Debug
        // std::cout << "Pub " << name << " messages sent: " << tracking_number << std::endl;
    }


    //Run the Publisher
    void run(int experiment_duration_sec)
    {
        // Sierra nevada period publishers: 10ms, 100ms and 500ms
        std::thread t_pub_parana   (&Hamburg::publish<Stamped3Float32>,  this, writer_parana_,   10,  12,    "parana");

        std::this_thread::sleep_for(std::chrono::seconds(experiment_duration_sec));

        // Stop threads and wait for them to finish
        run_threads = false;

        t_pub_parana.join();

        // Print stats
        listener_danube_.print_stats();
        listener_ganges_.print_stats();
        listener_nile_.print_stats();
        listener_tigris_.print_stats();
    }
};

int main(int argc, char** argv)
{
    int experiment_duration_sec = 300;
    std::cout << "Starting hamburg. Duration: " << experiment_duration_sec
              << " seconds.\n" << std::endl;

    Hamburg* hamburg = new Hamburg();

    if(hamburg->init()) {
        //Lets wait some time before start publising
        std::this_thread::sleep_for(std::chrono::seconds(20));
        hamburg->run(experiment_duration_sec);
    } else {
        std::cout << "Error at init stage." << std::endl;
    }

    delete hamburg;

    return 0;
}
