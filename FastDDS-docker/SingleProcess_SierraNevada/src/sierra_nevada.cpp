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
 * @file sierra_nevada.cpp
 * Replicates the performance benchamrk of sierra_nevada topology
 *
 */

// Include messages used in sierra_nevada topology
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

class SierraNevada
{
private:
    // Let's use only one participant and associate all pubs/subs to it
    DomainParticipant* participant_ {nullptr};

    // Declare publishers, writers, subscriber, readers, topics
    // and init by default to nullptr
    DECLARE_PUBSUB(amazon)
    DECLARE_PUBSUB(arkansas)
    DECLARE_PUBSUB(congo)
    DECLARE_PUBSUB(danube)
    DECLARE_PUBSUB(ganges)
    DECLARE_PUBSUB(lena)
    DECLARE_PUBSUB(mekong)
    DECLARE_PUBSUB(missouri)
    DECLARE_PUBSUB(nile)
    DECLARE_PUBSUB(parana)
    DECLARE_PUBSUB(salween)
    DECLARE_PUBSUB(tigris)
    DECLARE_PUBSUB(volga)

    // Danube has 4 subscriptions, parana has 2.
    // Let's declare those missing on previous macros
    Subscriber* sub_danube_2_    {nullptr};
    Subscriber* sub_danube_3_    {nullptr};
    Subscriber* sub_danube_4_    {nullptr};
    Subscriber* sub_parana_2_    {nullptr};

    DataReader* reader_danube_2_ {nullptr};
    DataReader* reader_danube_3_ {nullptr};
    DataReader* reader_danube_4_ {nullptr};
    DataReader* reader_parana_2_ {nullptr};

    // Objects to register the topic data type in the DomainParticipant.
    TypeSupport type_stamped12_float32_;
    TypeSupport type_stamped3_float32_;
    TypeSupport type_stamped4_float32_;
    TypeSupport type_stamped4_int32_;
    TypeSupport type_stamped9_float32_;
    TypeSupport type_stamped_int64_;
    TypeSupport type_stamped_vector_;

    // Create subscriber listeners
    SubListener<Stamped9Float32>  listener_amazon_;
    SubListener<Stamped4Int32>    listener_arkansas_;
    SubListener<Stamped4Int32>    listener_congo_;
    SubListener<StampedInt64>     listener_danube_;
    SubListener<StampedInt64>     listener_danube_2_;
    SubListener<StampedInt64>     listener_danube_3_;
    SubListener<StampedInt64>     listener_danube_4_;
    SubListener<Stamped4Int32>    listener_ganges_;
    SubListener<StampedVector>    listener_lena_;
    SubListener<StampedVector>    listener_mekong_;
    SubListener<StampedVector>    listener_missouri_;
    SubListener<Stamped4Int32>    listener_nile_;
    SubListener<Stamped3Float32>  listener_parana_;
    SubListener<Stamped3Float32>  listener_parana_2_;
    SubListener<Stamped12Float32> listener_salween_;
    SubListener<Stamped4Float32>  listener_tigris_;
    SubListener<StampedInt64>     listener_volga_;
    std::atomic_bool run_threads {true};

public:

    SierraNevada()
        : type_stamped12_float32_ (new Stamped12Float32PubSubType())
        , type_stamped3_float32_  (new Stamped3Float32PubSubType())
        , type_stamped4_float32_  (new Stamped4Float32PubSubType())
        , type_stamped4_int32_    (new Stamped4Int32PubSubType())
        , type_stamped9_float32_  (new Stamped9Float32PubSubType())
        , type_stamped_int64_     (new StampedInt64PubSubType())
        , type_stamped_vector_    (new StampedVectorPubSubType())
        , listener_amazon_("amazon")
        , listener_arkansas_("arkansas")
        , listener_congo_("congo")
        , listener_danube_("danube")
        , listener_danube_2_("danube_2")
        , listener_danube_3_("danube_3")
        , listener_danube_4_("danube_4")
        , listener_ganges_("ganges")
        , listener_lena_("lena")
        , listener_mekong_("mekong")
        , listener_missouri_("missouri")
        , listener_nile_("nile")
        , listener_parana_("parana")
        , listener_parana_2_("parana_2")
        , listener_salween_("salween")
        , listener_tigris_("tigris")
        , listener_volga_("volga")
    {
    }

    virtual ~SierraNevada()
    {
        // Delete data writers, publishers, data readers, subscribers, topics
        DESTRUCTOR(amazon)
        DESTRUCTOR(arkansas)
        DESTRUCTOR(congo)
        DESTRUCTOR(danube)
        DESTRUCTOR(ganges)
        DESTRUCTOR(lena)
        DESTRUCTOR(mekong)
        DESTRUCTOR(missouri)
        DESTRUCTOR(nile)
        DESTRUCTOR(parana)
        DESTRUCTOR(salween)
        DESTRUCTOR(tigris)
        DESTRUCTOR(volga)

        if(reader_danube_2_ != nullptr) { sub_danube_2_->delete_datareader(reader_danube_2_);}
        if(sub_danube_2_    != nullptr) { participant_->delete_subscriber(sub_danube_2_);}
        if(reader_danube_3_ != nullptr) { sub_danube_3_->delete_datareader(reader_danube_3_);}
        if(sub_danube_3_    != nullptr) { participant_->delete_subscriber(sub_danube_3_);}
        if(reader_danube_4_ != nullptr) { sub_danube_4_->delete_datareader(reader_danube_4_);}
        if(sub_danube_4_    != nullptr) { participant_->delete_subscriber(sub_danube_4_);}

        if(reader_parana_2_ != nullptr) { sub_parana_2_->delete_datareader(reader_parana_2_);}
        if(sub_parana_2_    != nullptr) { participant_->delete_subscriber(sub_parana_2_);}

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
        type_stamped12_float32_.register_type(participant_);
        type_stamped3_float32_.register_type(participant_);
        type_stamped4_float32_.register_type(participant_);
        type_stamped4_int32_.register_type(participant_);
        type_stamped9_float32_.register_type(participant_);
        type_stamped_int64_.register_type(participant_);
        type_stamped_vector_.register_type(participant_);

        // Init the publications/subscription Topics
        amazon_   = participant_->create_topic("amazon",   "Stamped9Float32",  TOPIC_QOS_DEFAULT);
        arkansas_ = participant_->create_topic("arkansas", "Stamped4Int32",    TOPIC_QOS_DEFAULT);
        congo_    = participant_->create_topic("congo",    "Stamped4Int32",    TOPIC_QOS_DEFAULT);
        danube_   = participant_->create_topic("danube",   "StampedInt64",     TOPIC_QOS_DEFAULT);
        ganges_   = participant_->create_topic("ganges",   "Stamped4Int32",    TOPIC_QOS_DEFAULT);
        lena_     = participant_->create_topic("lena",     "StampedVector",    TOPIC_QOS_DEFAULT);
        mekong_   = participant_->create_topic("mekong",   "StampedVector",    TOPIC_QOS_DEFAULT);
        missouri_ = participant_->create_topic("missouri", "StampedVector",    TOPIC_QOS_DEFAULT);
        nile_     = participant_->create_topic("nile",     "Stamped4Int32",    TOPIC_QOS_DEFAULT);
        parana_   = participant_->create_topic("parana",   "Stamped3Float32",  TOPIC_QOS_DEFAULT);
        salween_  = participant_->create_topic("salween",  "Stamped12Float32", TOPIC_QOS_DEFAULT);
        tigris_   = participant_->create_topic("tigris",   "Stamped4Float32",  TOPIC_QOS_DEFAULT);
        volga_    = participant_->create_topic("volga",    "StampedInt64",     TOPIC_QOS_DEFAULT);

        // Init the Publishers, DataWriters, Subscribers,DataReaders
        INIT_PUBSUB(amazon)
        INIT_PUBSUB(arkansas)
        INIT_PUBSUB(congo)
        INIT_PUBSUB(danube)
        INIT_PUBSUB(ganges)
        INIT_PUBSUB(lena)
        INIT_PUBSUB(mekong)
        INIT_PUBSUB(missouri)
        INIT_PUBSUB(nile)
        INIT_PUBSUB(parana)
        INIT_PUBSUB(salween)
        INIT_PUBSUB(tigris)
        INIT_PUBSUB(volga)

        sub_danube_2_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
        sub_danube_3_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
        sub_danube_4_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
        sub_parana_2_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);

        // All of them subscribe to same topic
        reader_danube_2_ = sub_danube_2_->create_datareader(danube_, DATAREADER_QOS_DEFAULT, &listener_danube_2_);
        reader_danube_3_ = sub_danube_3_->create_datareader(danube_, DATAREADER_QOS_DEFAULT, &listener_danube_3_);
        reader_danube_4_ = sub_danube_4_->create_datareader(danube_, DATAREADER_QOS_DEFAULT, &listener_danube_4_);
        reader_parana_2_ = sub_parana_2_->create_datareader(parana_, DATAREADER_QOS_DEFAULT, &listener_parana_2_);

        // The recently created pointers should not be null
        CHECK_NULL(amazon)
        CHECK_NULL(arkansas)
        CHECK_NULL(congo)
        CHECK_NULL(danube)
        CHECK_NULL(ganges)
        CHECK_NULL(lena)
        CHECK_NULL(mekong)
        CHECK_NULL(missouri)
        CHECK_NULL(nile)
        CHECK_NULL(parana)
        CHECK_NULL(salween)
        CHECK_NULL(tigris)
        CHECK_NULL(volga)

        if (sub_danube_2_ == nullptr)    { return false; }
        if (sub_danube_3_ == nullptr)    { return false; }
        if (sub_danube_4_ == nullptr)    { return false; }
        if (sub_parana_2_ == nullptr)    { return false; }
        if (reader_danube_2_ == nullptr) { return false; }
        if (reader_danube_3_ == nullptr) { return false; }
        if (reader_danube_4_ == nullptr) { return false; }
        if (reader_parana_2_ == nullptr) { return false; }

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
        std::thread t_pub_amazon   (&SierraNevada::publish<Stamped9Float32>,  this, writer_amazon_,   10,  36,    "amazon");
        std::thread t_pub_arkansas (&SierraNevada::publish<Stamped4Int32>,    this, writer_arkansas_, 100, 16,    "arkansas");
        std::thread t_pub_congo    (&SierraNevada::publish<Stamped4Int32>,    this, writer_congo_,    100, 16,    "congo");
        std::thread t_pub_danube   (&SierraNevada::publish<StampedInt64>,     this, writer_danube_,   10,  8,     "danube");
        std::thread t_pub_ganges   (&SierraNevada::publish<Stamped4Int32>,    this, writer_ganges_,   10,  16,    "ganges");
        std::thread t_pub_lena     (&SierraNevada::publish<StampedVector>,    this, writer_lena_,     100, 50,    "lena");
        std::thread t_pub_mekong   (&SierraNevada::publish<StampedVector>,    this, writer_mekong_,   500, 100,   "mekong");
        std::thread t_pub_missouri (&SierraNevada::publish<StampedVector>,    this, writer_missouri_, 100, 10000, "missouri");
        std::thread t_pub_nile     (&SierraNevada::publish<Stamped4Int32>,    this, writer_nile_,     10,  16,    "nile");
        std::thread t_pub_parana   (&SierraNevada::publish<Stamped3Float32>,  this, writer_parana_,   10,  12,    "parana");
        std::thread t_pub_salween  (&SierraNevada::publish<Stamped12Float32>, this, writer_salween_,  100, 48,    "salween");
        std::thread t_pub_tigris   (&SierraNevada::publish<Stamped4Float32>,  this, writer_tigris_,   10,  16,    "tigris");
        std::thread t_pub_volga    (&SierraNevada::publish<StampedInt64>,     this, writer_volga_,    500, 8,     "volga");

        std::this_thread::sleep_for(std::chrono::seconds(experiment_duration_sec));

        // Stop threads and wait for them to finish
        run_threads = false;

        t_pub_amazon.join();
        t_pub_arkansas.join();
        t_pub_congo.join();
        t_pub_danube.join();
        t_pub_ganges.join();
        t_pub_lena.join();
        t_pub_mekong.join();
        t_pub_missouri.join();
        t_pub_nile.join();
        t_pub_parana.join();
        t_pub_salween.join();
        t_pub_tigris.join();
        t_pub_volga.join();

        // Print stats
        listener_amazon_.print_stats();
        listener_arkansas_.print_stats();
        listener_congo_.print_stats();
        listener_danube_.print_stats();
        listener_danube_2_.print_stats();
        listener_danube_3_.print_stats();
        listener_danube_4_.print_stats();
        listener_ganges_.print_stats();
        listener_lena_.print_stats();
        listener_mekong_.print_stats();
        listener_missouri_.print_stats();
        listener_nile_.print_stats();
        listener_parana_.print_stats();
        listener_parana_2_.print_stats();
        listener_salween_.print_stats();
        listener_tigris_.print_stats();
        listener_volga_.print_stats();
    }
};

int main(int argc, char** argv)
{
    int experiment_duration_sec = 100;
    std::cout << "Starting sierra_nevada. Duration: " << experiment_duration_sec
              << " seconds.\n" << std::endl;

    SierraNevada* sierra_nevada = new SierraNevada();

    if(sierra_nevada->init()) {
        sierra_nevada->run(experiment_duration_sec);
    } else {
        std::cout << "Error at init stage." << std::endl;
    }

    std::cout << "\nFinished. Begin destruction.\n" << std::endl;

    delete sierra_nevada;

    return 0;
}
