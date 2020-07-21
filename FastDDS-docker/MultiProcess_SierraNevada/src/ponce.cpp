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
 * @file ponce.cpp
 * Replicates the performance benchmark of ponce topology
 *
 */

// Include messages used in ponce topology
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

class Ponce
{
private:
    // Let's use only one participant and associate all pubs/subs to it
    DomainParticipant* participant_ {nullptr};

    // Declare publishers, writers, subscriber, readers, topics
    // and init by default to nullptr
    DECLARE_PUB(congo)
    DECLARE_PUB(mekong)
    DECLARE_SUB(missouri)
    DECLARE_SUB(volga)

    // Danube has 4 subscriptions, parana has 2.
    // Let's declare those missing on previous macros
    Subscriber* sub_danube_3_    {nullptr};
    DataReader* reader_danube_3_ {nullptr};
    Topic* danube_ {nullptr};

    // Objects to register the topic data type in the DomainParticipant.
    TypeSupport type_stamped4_int32_;
    TypeSupport type_stamped_int64_;
    TypeSupport type_stamped_vector_;

    // Create subscriber listeners
    SubListener<StampedInt64>     listener_danube_3_;
    SubListener<StampedVector>    listener_missouri_;
    SubListener<StampedInt64>     listener_volga_;
    std::atomic_bool run_threads {true};

public:

    Ponce()
        : type_stamped4_int32_    (new Stamped4Int32PubSubType())
        , type_stamped_int64_     (new StampedInt64PubSubType())
        , type_stamped_vector_    (new StampedVectorPubSubType())
        , listener_danube_3_("danube_3")
        , listener_missouri_("missouri")
        , listener_volga_("volga")
    {
    }

    virtual ~Ponce()
    {
        // Delete data writers, publishers, data readers, subscribers, topics
        DESTRUCTOR_PUB(congo)
        DESTRUCTOR_PUB(mekong)
        DESTRUCTOR_SUB(missouri)
        DESTRUCTOR_SUB(volga)

        if(reader_danube_3_ != nullptr) { sub_danube_3_->delete_datareader(reader_danube_3_);}
        if(sub_danube_3_    != nullptr) { participant_->delete_subscriber(sub_danube_3_);}

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
        type_stamped4_int32_.register_type(participant_);
        type_stamped_int64_.register_type(participant_);
        type_stamped_vector_.register_type(participant_);

        // Init the publications/subscription Topics
        congo_    = participant_->create_topic("congo",    "Stamped4Int32",    TOPIC_QOS_DEFAULT);
        danube_   = participant_->create_topic("danube",   "StampedInt64",     TOPIC_QOS_DEFAULT);
        mekong_   = participant_->create_topic("mekong",   "StampedVector",    TOPIC_QOS_DEFAULT);
        missouri_ = participant_->create_topic("missouri", "StampedVector",    TOPIC_QOS_DEFAULT);
        volga_    = participant_->create_topic("volga",    "StampedInt64",     TOPIC_QOS_DEFAULT);

        // Init the Publishers, DataWriters, Subscribers,DataReaders
        INIT_PUB(congo)
        INIT_PUB(mekong)
        INIT_SUB(missouri)
        INIT_SUB(volga)

        sub_danube_3_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);

        // All of them subscribe to same topic
        reader_danube_3_ = sub_danube_3_->create_datareader(danube_, DATAREADER_QOS_DEFAULT, &listener_danube_3_);

        // The recently created pointers should not be null

        if (sub_danube_3_ == nullptr)    { return false; }
        if (reader_danube_3_ == nullptr) { return false; }

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
        std::thread t_pub_congo    (&Ponce::publish<Stamped4Int32>,    this, writer_congo_,    100, 16,    "congo");
        std::thread t_pub_mekong   (&Ponce::publish<StampedVector>,    this, writer_mekong_,   500, 100,   "mekong");

        std::this_thread::sleep_for(std::chrono::seconds(experiment_duration_sec));

        // Stop threads and wait for them to finish
        run_threads = false;

        t_pub_congo.join();
        t_pub_mekong.join();

        // Print stats
        listener_danube_3_.print_stats();
        listener_missouri_.print_stats();
        listener_volga_.print_stats();
    }
};

int main(int argc, char** argv)
{
    int experiment_duration_sec = 300;
    std::cout << "Starting ponce. Duration: " << experiment_duration_sec
              << " seconds.\n" << std::endl;

    Ponce* ponce = new Ponce();

    if(ponce->init()) {
        //Lets wait some time before start publising
        std::this_thread::sleep_for(std::chrono::seconds(20));
        ponce->run(experiment_duration_sec);
    } else {
        std::cout << "Error at init stage." << std::endl;
    }

    delete ponce;

    return 0;
}
