// These macros really help the code to avoid being huge, and will also help
// in case of increasing the size of the topology
// as currently we're testing sierra nevada, the smallest.

/***************************************************************************************************************************/
// For single process topology
#define DECLARE_PUBSUB(Name)                  \
  Publisher*  pub_ ## Name ## _    {nullptr}; \
  DataWriter* writer_ ## Name ## _ {nullptr}; \
  Subscriber* sub_ ## Name ## _    {nullptr}; \
  DataReader* reader_ ## Name ## _ {nullptr}; \
  Topic* Name ## _ {nullptr};

// For multi process topology
#define DECLARE_PUB(Name)                     \
  Publisher*  pub_ ## Name ## _    {nullptr}; \
  DataWriter* writer_ ## Name ## _ {nullptr}; \
  Topic* Name ## _ {nullptr};

#define DECLARE_SUB(Name)                     \
  Subscriber* sub_ ## Name ## _    {nullptr}; \
  DataReader* reader_ ## Name ## _ {nullptr}; \
  Topic* Name ## _ {nullptr};

/***************************************************************************************************************************/

// For single process topology
#define INIT_PUBSUB(Name)                                                                        \
  pub_ ## Name ##_ =         participant_->create_publisher( PUBLISHER_QOS_DEFAULT, nullptr);    \
  writer_ ## Name ##_ =  pub_ ## Name ##_->create_datawriter( Name ##_, DATAWRITER_QOS_DEFAULT); \
  sub_ ## Name ##_ =         participant_->create_subscriber( SUBSCRIBER_QOS_DEFAULT, nullptr);  \
  reader_ ## Name ##_ =  sub_ ## Name ##_->create_datareader( Name ##_, DATAREADER_QOS_DEFAULT, &listener_ ## Name ##_);

// For multi process topology
#define INIT_PUB(Name)                                                                           \
  pub_ ## Name ##_ =         participant_->create_publisher( PUBLISHER_QOS_DEFAULT, nullptr);    \
  writer_ ## Name ##_ =  pub_ ## Name ##_->create_datawriter( Name ##_, DATAWRITER_QOS_DEFAULT); \
  if (pub_ ## Name ##_ == nullptr)    { return false; } \
  if (writer_ ## Name ##_ == nullptr) { return false; }

#define INIT_SUB(Name)                                                                                                   \
  sub_ ## Name ##_ =         participant_->create_subscriber( SUBSCRIBER_QOS_DEFAULT, nullptr);                          \
  reader_ ## Name ##_ =  sub_ ## Name ##_->create_datareader( Name ##_, DATAREADER_QOS_DEFAULT, &listener_ ## Name ##_); \
  if (sub_ ## Name ##_ == nullptr)    { return false; }                                                                  \
  if (reader_ ## Name ##_ == nullptr) { return false; }

/***************************************************************************************************************************/

// For single process topology
#define CHECK_NULL(Name)                                \
  if (Name ##_ == nullptr)            { return false; } \
  if (pub_ ## Name ##_ == nullptr)    { return false; } \
  if (writer_ ## Name ##_ == nullptr) { return false; } \
  if (sub_ ## Name ##_ == nullptr)    { return false; } \
  if (reader_ ## Name ##_ == nullptr) { return false; }

/***************************************************************************************************************************/
// For single process topology
#define DESTRUCTOR(Name)                                                                               \
  if (writer_ ## Name ## _ != nullptr) { pub_ ## Name ## _->delete_datawriter(writer_ ## Name ## _);}  \
  if (pub_ ## Name ## _    != nullptr) { participant_->delete_publisher(pub_ ## Name ## _);}           \
  if (reader_ ## Name ## _ != nullptr) { sub_ ## Name ## _->delete_datareader(reader_ ## Name ## _);}  \
  if (sub_ ## Name ## _    != nullptr) { participant_->delete_subscriber(sub_ ## Name ## _);}          \
  if (Name ## _            != nullptr) { participant_->delete_topic(Name ## _);}

// For multi process topology
#define DESTRUCTOR_PUB(Name)                                                                           \
  if (writer_ ## Name ## _ != nullptr) { pub_ ## Name ## _->delete_datawriter(writer_ ## Name ## _);}  \
  if (pub_ ## Name ## _    != nullptr) { participant_->delete_publisher(pub_ ## Name ## _);}           \
  if (Name ## _            != nullptr) { participant_->delete_topic(Name ## _);}
  
#define DESTRUCTOR_SUB(Name)                                                                           \
  if (reader_ ## Name ## _ != nullptr) { sub_ ## Name ## _->delete_datareader(reader_ ## Name ## _);}  \
  if (sub_ ## Name ## _    != nullptr) { participant_->delete_subscriber(sub_ ## Name ## _);}          \
  if (Name ## _            != nullptr) { participant_->delete_topic(Name ## _);}
  