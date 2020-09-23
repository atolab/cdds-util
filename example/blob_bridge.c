#include <string.h>
#include "cdds/cdds_util.h"


int main(int argc, char *argv[])
{
  char* partition = NULL;

  if (argc < 5) {
    printf("USAGE:\n\tbbridge <in-topic-name> <out-topic-name> <type_name> <1|0, 1 => keyless and 0 => keyed> [<partition>]\n");
    exit(1);
  }
  if (argc > 5) {
    partition = argv[4];
  }
  int keyless;
  sscanf(argv[4], "%d", &keyless);
  printf("keyless = %d\n", keyless);
  dds_return_t rc;

  const dds_entity_t dp = dds_create_participant (DDS_DOMAIN_DEFAULT, NULL, NULL);
  struct ddsi_sertopic *in_st = cdds_create_blob_sertopic(dp, argv[1], argv[3], keyless == 1);
  struct ddsi_sertopic *out_st = cdds_create_blob_sertopic(dp, argv[2], argv[3], keyless == 1);
  const dds_entity_t in_tp = dds_create_topic_generic(dp, &in_st, 0, 0, 0);
  const dds_entity_t out_tp = dds_create_topic_generic(dp, &out_st, 0, 0, 0);

  dds_qos_t *qos = NULL;
  if (partition != NULL) {

    qos = dds_create_qos();
    dds_qset_partition1(qos, partition);
  }

  const dds_entity_t rd = dds_create_reader (dp, in_tp, qos, NULL);
  const dds_entity_t wr = dds_create_writer (dp, out_tp, qos, NULL);

  do {
    struct cdds_ddsi_payload *zp = NULL;
    dds_sample_info_t si;
    rc = cdds_take_blob(rd, &zp, &si);
    printf("Received %d samples\n", rc);
    if (rc > 0) {
      printf("Payload:\n Size = %d\n", (int)zp->size);
      for (int i = 0; i < zp->size; ++i) {
        printf("%d ", (int)zp->payload[i]);
      }
      printf("Trying to write...\n");
      // Re-create serdata just to show what one would have to do when receiving this
      // out-of band from DDS.
      // cdds_make_serdata_from_blob(out_tp, zp->payload, zp->size);

      struct cdds_ddsi_payload *fwd = cdds_ddsi_payload_create(out_st, zp->kind, zp->payload, zp->size);
      dds_writecdr(wr, (struct ddsi_serdata*)fwd);
      printf("\n");
    }

    dds_sleepfor (DDS_MSECS (1000));
  } while(true);

  rc = dds_delete (dp);
}
