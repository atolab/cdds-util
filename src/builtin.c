#include "cdds/cdds_builtin.h"


static const uint32_t NameValue_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (NameValue, name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (NameValue, value),
  DDS_OP_RTS
};

const dds_topic_descriptor_t NameValue_desc =
{
  sizeof (NameValue),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "NameValue",
  NULL,
  3,
  NameValue_ops,
  "<MetaData version=\"1.0.0\"><Struct name=\"NameValue\"><Member name=\"name\"><String/></Member><Member name=\"value\"><String/></Member></Struct></MetaData>"
};


static const uint32_t KeyValue_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (KeyValue, key),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (KeyValue, value),
  DDS_OP_RTS
};

const dds_topic_descriptor_t KeyValue_desc =
{
  sizeof (KeyValue),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "KeyValue",
  NULL,
  3,
  KeyValue_ops,
  "<MetaData version=\"1.0.0\"><Struct name=\"KeyValue\"><Member name=\"key\"><String/></Member><Member name=\"value\"><String/></Member></Struct></MetaData>"
};
