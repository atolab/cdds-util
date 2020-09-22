#include "cdds/cdds_util.h"


int main(int argc, char *argv[])
{
  char* partition = NULL;

  if (argc < 4) {
    printf("USAGE:\n\tbsub <topic-name> <type_name> <1|0, 1 => keyless and 0 => keyed> [<partition>]\n");
    exit(1);
  }
  if (argc > 4) {
    partition = argv[4];
  }
  int keyless;
  sscanf(argv[3], "%d", &keyless);
  printf("keyless = %d\n", keyless);
  dds_return_t rc;

  const dds_entity_t dp = dds_create_participant (DDS_DOMAIN_DEFAULT, NULL, NULL);
  const dds_entity_t tp = cdds_create_blob_topic(dp, argv[1], argv[2], keyless == 1);

  dds_qos_t *qos = NULL;
  if (partition != NULL) {

    qos = dds_create_qos();
    dds_qset_partition1(qos, partition);
  }

  const dds_entity_t rd = dds_create_reader (dp, tp, qos, NULL);

  do
  {
    struct cdds_ddsi_payload *zp = NULL;
    dds_sample_info_t si;
    rc = cdds_take_blob(rd, &zp, &si);
    printf("Received %d samples\n", rc);
    if (rc > 0) {
      printf("Payload:\n Size = %d\n", (int)zp->size);
      for (int i = 0; i < zp->size; ++i) {
        printf("%d ", (int)zp->payload[i]);
      }
      printf("\n");
    }
    dds_sleepfor (DDS_MSECS (1000));
  } while(true);

  rc = dds_delete (dp);
}
