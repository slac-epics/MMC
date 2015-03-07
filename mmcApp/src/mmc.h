#define MAX_MSG_SIZE 64
#define OK            0

typedef union
{
    epicsInt16 All;
    struct
    {
        unsigned int RA_LLS         :1; // 1=Active, 0=Not active
        unsigned int RA_HLS         :1; // 1=Active, 0=Not active
        unsigned int RA_PRGM        :1; // 1=a program is running, 0=No program
        unsigned int RA_STOPPED     :1;
        unsigned int RA_DEC         :1;
        unsigned int RA_VELO        :1;
        unsigned int RA_ACC         :1;
        unsigned int RA_ERR         :1; // 1=One or more errors, 0=No error
    } Bits;
} mmc_status;

struct mmca_info
{
    CALLBACK            callback;
    struct mmcaRecord  *precord;

    epicsMutex         *uMutex;
    short               init;

    epicsUInt16         sta;
    float               vrt;
    float               tpos;
    float               epos;
    char                rsp[MAX_MSG_SIZE];
    short               newData;

    epicsMutex         *lMutex;
    short               mLength;
    char               *sAddr;
    int                 cIndex;
    bool                newMsg;
};

typedef union
{
    epicsUInt32 All;
    struct
    {
        unsigned int RA_DIRECTION   :1; // (last) 0=Negative, 1=Positive
        unsigned int RA_DONE        :1; // a motion is complete
        unsigned int RA_PLUS_LS     :1; // plus limit switch has been hit
        unsigned int RA_HOME        :1; // home signal is on
        unsigned int RA_SM          :1; // continue on stall detect
        unsigned int EA_POSITION    :1; // position maintenence enabled
        unsigned int EA_SLIP_STALL  :1; // slip/stall detected
        unsigned int EA_HOME        :1; // encoder home signal on
        unsigned int RA_EE          :1; // encoder enable
        unsigned int RA_PROBLEM     :1; // driver stopped polling
        unsigned int RA_MOVING      :1; // non-zero velocity present
        unsigned int GAIN_SUPPORT   :1; // support closed-loop position control
        unsigned int RA_COMM_ERR    :1; // controller communication error
        unsigned int RA_MINUS_LS    :1; // minus limit switch has been hit
        unsigned int RA_HOMED       :1; // axis has been homed
        unsigned int RA_ERR         :7; // error number
        unsigned int RA_STALL       :1; // stall detected
        unsigned int RA_POWERUP     :1; // power-cycled
        unsigned int RA_NE          :1; // numeric enable
        unsigned int RA_BY0         :1; // MCode not running (BY = 0)
        unsigned int NA             :5; // un-used bits
        unsigned int NOT_INIT       :1; // initialization not finished
    } Bits;
} motor_status;

/* Bit map of changed fields */
typedef union
{
    epicsUInt32 All;
    struct
    {
        unsigned int M_STA      :1;
        unsigned int M_VRT      :1;
        unsigned int M_VAL      :1;
        unsigned int M_DVAL     :1;
        unsigned int M_RVAL     :1;
        unsigned int M_RRBV     :1;
        unsigned int M_DRBV     :1;
        unsigned int M_RBV      :1;
        unsigned int M_DIFF     :1;
        unsigned int M_MIP      :1;
        unsigned int M_MOVN     :1;
        unsigned int M_DMOV     :1;
        unsigned int M_RCNT     :1;
        unsigned int M_MISS     :1;
        unsigned int M_RLLS     :1;
        unsigned int M_RHLS     :1;
        unsigned int M_LLS      :1;
        unsigned int M_HLS      :1;
        unsigned int M_LVIO     :1;
        unsigned int M_MSTA     :1;
    } Bits;
} changed_fields;

#define   MARK(FIELD) { changed_fields temp; temp.All = prec->cmap;            \
                        temp.Bits.FIELD = 1; prec->cmap = temp.All; }

#define MARKED(FIELD) ( cmap.Bits.FIELD )

#define UNMARK_ALL      prec->cmap = 0

/*** All db_post_events() calls set both VALUE and LOG bits ***/
#define DBE_VAL_LOG (unsigned int) (DBE_VALUE | DBE_LOG)

#define NINT(f) (long)((f)>0 ? (f)+0.5 : (f)-0.5)             // Nearest integer

