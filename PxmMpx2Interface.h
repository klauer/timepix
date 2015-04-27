
#ifndef SoPhyMedipixNative_SPMpx2Interface_h
#define SoPhyMedipixNative_SPMpx2Interface_h

#include "mpxhw.h"


class PxmMpx2Interface : public Mpx2Interface {
public:
    void *libHandle;
    PxmMpx2Interface(Mpx2Interface *);
    ~PxmMpx2Interface();
};

#endif
