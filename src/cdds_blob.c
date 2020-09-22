#include "cdds/cdds_util.h"

struct cdds_ddsi_payload* cdds_ddsi_payload_create(struct ddsi_sertopic *st, enum ddsi_serdata_kind kind, unsigned char* buf, size_t size) {
  struct cdds_ddsi_payload* p = (struct cdds_ddsi_payload*)malloc(sizeof(struct cdds_ddsi_payload));
  ddsi_serdata_init(&p->sd, st, kind);
  p->kind = kind;
  p->payload = buf;
  p->size = size;
  return p;
}

void cdds_ddsi_payload_free(struct cdds_ddsi_payload* p) {
  free(p);
}

unsigned char* cdds_ddsi_payload_get(struct cdds_ddsi_payload* p) {
  return p->payload;
}

size_t cdds_ddsi_payload_len(struct cdds_ddsi_payload* p) {
  return p->size;
}

enum ddsi_serdata_kind cdds_ddsi_payload_kind(struct cdds_ddsi_payload* p) {
  return p->kind;
}


static bool cdds_sertopic_equal (const struct ddsi_sertopic *acmn, const struct ddsi_sertopic *bcmn)
{
  // no fields in stp beyond the common ones, and those are all checked for equality before this function is called
  (void) acmn; (void) bcmn;
  return true;
}

static uint32_t cdds_sertopic_hash (const struct ddsi_sertopic *tpcmn)
{
  // nothing beyond the common fields
  (void) tpcmn;
  return 0;
}


static void cdds_sertopic_free(struct ddsi_sertopic * tpcmn) {
  ddsi_sertopic_fini(tpcmn);
}


static void cdds_sertopic_zero_samples(const struct ddsi_sertopic * d, void * samples, size_t count) {
  (void)d;
  (void)samples;
  (void)count;
  /* Not using code paths that rely on the samples getting zero'd out */
}


static void cdds_sertopic_realloc_samples(
  void ** ptrs, const struct ddsi_sertopic * d,
  void * old, size_t oldcount, size_t count)
{
  (void)(ptrs);
  (void)(d);
  (void)(old);
  (void)(oldcount);
  (void)(count);
  /* Not using code paths that rely on this (loans, dispose, unregister with instance handle,
     content filters) */
  abort();
}

static void cdds_sertopic_free_samples(
  const struct ddsi_sertopic * d, void ** ptrs, size_t count,
  dds_free_op_t op)
{
  (void)(d);    // unused
  (void)(ptrs);    // unused
  (void)(count);    // unused
  /* Not using code paths that rely on this (dispose, unregister with instance handle, content
     filters) */
  assert(!(op & DDS_FREE_ALL_BIT));
  (void) op;
}

static const struct ddsi_sertopic_ops cdds_sertopic_ops = {
  .free = cdds_sertopic_free,
  .zero_samples = cdds_sertopic_zero_samples,
  .realloc_samples = cdds_sertopic_realloc_samples,
  .free_samples = cdds_sertopic_free_samples,
  .equal = cdds_sertopic_equal,
  .hash = cdds_sertopic_hash
};

static bool cdds_serdata_eqkey(const struct ddsi_serdata * a, const struct ddsi_serdata * b)
{
  (void)(a);
  (void)(b);
  /* ROS 2 doesn't do keys in a meaningful way yet */
  printf("Called <cdds_serdata_eqkey>\n");
  return true;
}

static uint32_t cdds_serdata_size(const struct ddsi_serdata * sd)
{
  printf("Called <cdds_serdata_size>\n");
  struct cdds_ddsi_payload * zp = (struct cdds_ddsi_payload *)sd;
  return zp->size;
}

static void cdds_serdata_free(struct ddsi_serdata * sd)
{
  printf("Called <cdds_serdata_free>\n");
  struct cdds_ddsi_payload * zp = (struct cdds_ddsi_payload *)sd;
  free(zp->payload);
  zp->size = 0;
  // TODO: verify that dds_fini does not need to be called on zp->sd
  free(zp);
}

static struct ddsi_serdata *cdds_serdata_from_ser_iov (const struct ddsi_sertopic *tpcmn, enum ddsi_serdata_kind kind, ddsrt_msg_iovlen_t niov, const ddsrt_iovec_t *iov, size_t size)
{
  printf("==> <cdds_serdata_from_ser_iov> for %s -- size %zu\n", tpcmn->name, size);
  struct cdds_ddsi_payload *zp = (struct cdds_ddsi_payload *)malloc(sizeof(struct cdds_ddsi_payload));
  ddsi_serdata_init(&zp->sd, tpcmn, kind);
  zp->size = size;
  zp->kind = kind;
  zp->payload = 0;
  switch (kind) {
    case SDK_KEY:
    case SDK_DATA:
      zp->payload = malloc(size);
      int offset = 0;
      int csize = 0;
      for (int i = 0; i < niov; ++i) {
        csize += iov[i].iov_len;
        assert(csize <= size);
        memcpy(zp->payload + offset, iov[i].iov_base, iov[i].iov_len);
        offset += iov[i].iov_len;
      }
      break;
    case SDK_EMPTY:
      break;
  }
  return (struct ddsi_serdata *)zp;
}

static struct ddsi_serdata *cdds_serdata_from_ser (const struct ddsi_sertopic *tpcmn, enum ddsi_serdata_kind kind, const struct nn_rdata *fragchain, size_t size)
{
  printf("Called <cdds_serdata_from_ser> for %s\n", tpcmn->name);
  // This currently assumes that there is only one fragment.
  assert (fragchain->nextfrag == NULL);
  ddsrt_iovec_t iov = {
    .iov_base = NN_RMSG_PAYLOADOFF (fragchain->rmsg, NN_RDATA_PAYLOAD_OFF (fragchain)),
    .iov_len = fragchain->maxp1 // fragchain->min = 0 for first fragment, by definition
  };
  return cdds_serdata_from_ser_iov (tpcmn, kind, 1, &iov, size);
}

static struct ddsi_serdata *cdds_serdata_to_topicless (const struct ddsi_serdata *sd) {
  printf("Called <cdds_serdata_to_topicless> \n");
  return ddsi_serdata_ref(sd);
}

static struct ddsi_serdata *cdds_to_ser_ref (const struct ddsi_serdata *serdata_common, size_t cdr_off, size_t cdr_sz, ddsrt_iovec_t *ref) {
  printf("Called <cdds_to_ser_ref> \n");
  struct cdds_ddsi_payload *pl = (struct cdds_ddsi_payload *)serdata_common;
  // TODO: Address fragmentation
  assert (cdr_off == 0 && cdr_sz == pl->size);

  ref->iov_base = pl->payload;
  ref->iov_len = pl->size;
  return ddsi_serdata_ref(serdata_common);
}

static void cdds_to_ser (const struct ddsi_serdata *serdata_common, size_t off, size_t sz, void *buf) {
  printf("Called <cdds_to_ser_ref> \n");
  struct cdds_ddsi_payload *pl = (struct cdds_ddsi_payload *)serdata_common;
  memcpy(buf, pl->payload, pl->size);
}

static void cdds_to_ser_unref (struct ddsi_serdata *serdata_common, const ddsrt_iovec_t *ref) {
  printf("Called <sd_to_ser_unref> \n");
  (void)serdata_common;
  free(ref->iov_base);
}

static const struct ddsi_serdata_ops cdds_serdata_ops = {
  .get_size = cdds_serdata_size,
  .eqkey = cdds_serdata_eqkey,
  .from_ser = cdds_serdata_from_ser,
  .from_ser_iov = cdds_serdata_from_ser_iov,
  .to_topicless = cdds_serdata_to_topicless,
  .to_ser = cdds_to_ser,
  .to_ser_ref = cdds_to_ser_ref,
  .to_ser_unref = cdds_to_ser_unref
};

struct ddsi_sertopic* cdds_create_blob_sertopic(dds_entity_t dp, char *topic_name, char* type_name, bool is_keyless) {
  struct ddsi_sertopic *st = (struct ddsi_sertopic*) malloc(sizeof(struct ddsi_sertopic));
  ddsi_sertopic_init (st, topic_name, type_name, &cdds_sertopic_ops, &cdds_serdata_ops, is_keyless);
  return st;
}

dds_entity_t cdds_create_blob_topic(dds_entity_t dp, char *topic_name, char* type_name, bool is_keyless) {
  struct ddsi_sertopic *st = (struct ddsi_sertopic*) malloc(sizeof(struct ddsi_sertopic));
  ddsi_sertopic_init (st, topic_name, type_name, &cdds_sertopic_ops, &cdds_serdata_ops, is_keyless);
  return dds_create_topic_generic (dp, &st, NULL, NULL, NULL);
}

int cdds_take_blob(dds_entity_t rd, struct cdds_ddsi_payload** sample, dds_sample_info_t * si) {
  struct ddsi_serdata **sd = (struct ddsi_serdata **)sample;
  return dds_takecdr(rd, sd, 1, si, DDS_ANY_STATE);
}
