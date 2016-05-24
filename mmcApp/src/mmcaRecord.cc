#define VERSION 1.0

#include <algorithm>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <alarm.h>
#include <dbDefs.h>
#include <callback.h>
#include <dbStaticLib.h>
#include <dbAccess.h>
#include <dbScan.h>
#include <recGbl.h>
#include <recSup.h>
#include <dbEvent.h>
#include <devSup.h>
#include <math.h>
#include <time.h>
#include <errlog.h>

#define GEN_SIZE_OFFSET
#include "mmcaRecord.h"
#undef GEN_SIZE_OFFSET

#include "epicsMessageQueue.h"
#include "epicsExport.h"

#include "mmc.h"


/*** Forward references ***/


/*** Record Support Entry Table (RSET) functions ***/

static long init_record  ( dbCommon *precord, int pass        );
static long process      ( dbCommon *precord                  );
static long special      ( dbAddr   *pDbAddr, int after       );
static long cvt_dbaddr   ( dbAddr   *pDbAddr                  );
static long get_precision( dbAddr   *pDbAddr, long *precision );


rset mmcaRSET =
{
    RSETNUMBER,
    NULL,
    NULL,
    (RECSUPFUN) init_record,
    (RECSUPFUN) process,
    (RECSUPFUN) special,
    NULL,
    (RECSUPFUN) cvt_dbaddr,
    NULL,
    NULL,
    NULL,
    (RECSUPFUN) get_precision,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL
};

extern "C" { epicsExportAddress( rset, mmcaRSET ); }


#define MIP_DONE     0x0000    // No motion is in progress
#define MIP_MOVE     0x0001    // A move not resulting from Jog* or Hom*
#define MIP_RETRY    0x0002    // A retry is in progress
#define MIP_NEW      0x0004    // Stop current move for a new move
#define MIP_BL       0x0008    // Done moving, now take out backlash
#define MIP_HOMF     0x0010    // A home-forward command is in progress
#define MIP_HOMR     0x0020    // A home-reverse command is in progress
#define MIP_HOMB     0x0040    // Back off from the limit switch
#define MIP_HOMC     0x0080    // Creep to the limit switch
#define MIP_HOME     (MIP_HOMF | MIP_HOMR)
#define MIP_JOGF     0x0100    // Jog forward
#define MIP_JOGR     0x0200    // Jog backward
#define MIP_JOG      (MIP_JOGF | MIP_JOGR)
#define MIP_MLP      0x0400    // Move to Positive Limit
#define MIP_MLN      0x0800    // Move to Negative Limit
#define MIP_M2L      (MIP_MLP  | MIP_MLN )
#define MIP_PAUSE    0x1000    // Move is being paused
#define MIP_STOP     0x8000    // We're trying to stop.  If a home command
                               // is issued when the motor is moving, we
                               // stop the motor first


static long init_axis   ( mmcaRecord *precord                             );
static void new_move    ( mmcaRecord *precord                             );
static void do_move     ( mmcaRecord *precord                             );
static void post_fields ( mmcaRecord *precord, unsigned short alarm_mask,
                                               unsigned short all         );

static long update_field( struct mmca_info *mInfo                         );

static void getResponse ( struct mmca_info *mInfo                         );

static long log_msg  ( mmcaRecord *prec, int dlvl, const char *fmt, ...   );
static void post_msgs( mmcaRecord *prec                                   );


extern epicsMessageQueue *mmcCmd;
extern epicsMessageQueue *mmcRsp[];


/*** Debugging ***/

volatile int mmcaRecordDebug = 0;

extern "C" { epicsExportAddress( int, mmcaRecordDebug ); }

using namespace std;


/******************************************************************************/
static long init_record( dbCommon *precord, int pass )
{
    mmcaRecord  *prec = (mmcaRecord *)precord;
    mmca_info   *mInfo;

    long         status = 0;

    if ( pass == 0 ) return( status );

    mInfo = (mmca_info *)malloc( sizeof(mmca_info) );
    mInfo->precord = prec;
    mInfo->uMutex  = new epicsMutex();

    mInfo->newData = 0;

    mInfo->lMutex  = new epicsMutex();
    mInfo->mLength = 61;
    mInfo->cIndex  = 0;
    mInfo->newMsg  = 0;
    mInfo->sAddr   = (char *)calloc( 8*61, sizeof(char) );

    prec->loga     = mInfo->sAddr;
    prec->logb     = mInfo->sAddr + mInfo->mLength * 1;
    prec->logc     = mInfo->sAddr + mInfo->mLength * 2;
    prec->logd     = mInfo->sAddr + mInfo->mLength * 3;
    prec->loge     = mInfo->sAddr + mInfo->mLength * 4;
    prec->logf     = mInfo->sAddr + mInfo->mLength * 5;
    prec->logg     = mInfo->sAddr + mInfo->mLength * 6;
    prec->logh     = mInfo->sAddr + mInfo->mLength * 7;

    prec->dpvt     = mInfo;

    gethostname( prec->host, 60                   );
    strcpy     ( prec->iocn, getenv("EPICS_NAME") );

    epicsThreadCreate( prec->name, epicsThreadPriorityMedium,
                       epicsThreadGetStackSize(epicsThreadStackMedium),
                       (EPICSTHREADFUNC)getResponse, (void *)mInfo );

    init_axis( prec );

    return( status );
}

/******************************************************************************/
static long init_axis( mmcaRecord *prec )
{
    mmca_info       *mInfo = (mmca_info *)prec->dpvt;
    char             cmd[MAX_MSG_SIZE];
    unsigned int     clen;
    unsigned short   alarm_mask;

    motor_status     msta;

    mInfo->uMutex->lock();

    mInfo->init        = -1;

    msta.All           =  0;
    msta.Bits.NOT_INIT =  1;

    // soft reset
//  clen = sprintf( cmd, "%dRST",  prec->axis );
//  mmcCmd->send( cmd, clen );

    // read the firmware version
    clen = sprintf( cmd, "%dVER?", prec->axis );
    mmcCmd->send( cmd, clen );

    // read and clear errors
//  clen = sprintf( cmd, "%dERR?", prec->axis );
//  mmcCmd->send( cmd, clen );

    // ac/deceleration and velocity
    clen = sprintf( cmd, "%dAMX%.3f", prec->axis, prec->amax );
    mmcCmd->send( cmd, clen );

    clen = sprintf( cmd, "%dACC%.3f", prec->axis, prec->accl );
    mmcCmd->send( cmd, clen );

    clen = sprintf( cmd, "%dDEC%.3f", prec->axis, prec->accl );
    mmcCmd->send( cmd, clen );

    clen = sprintf( cmd, "%dVEL%.3f", prec->axis, prec->velo );
    mmcCmd->send( cmd, clen );

    // encoder etc
    clen = sprintf( cmd, "%dEAD%d",   prec->axis, prec->ead  );
    mmcCmd->send( cmd, clen );

    clen = sprintf( cmd, "%dEPL%d",   prec->axis, prec->epl  );
    mmcCmd->send( cmd, clen );

    if ( prec->ead == mmcaEAD_Digital ) // set the digital encoder resolution
    {
        clen = sprintf( cmd, "%dENC%.3f", prec->axis, prec->eres*1000 );
        mmcCmd->send( cmd, clen );
    }
    else                                   // read the analog encoder resolution
    {
        clen = sprintf( cmd, "%dENC?",    prec->axis                  );
        mmcCmd->send( cmd, clen );
    }

    // feedback mode
    clen = sprintf( cmd, "%dFBK%d",   prec->axis, prec->fbk  );
    mmcCmd->send( cmd, clen );

    // limit switche etc
    clen = sprintf( cmd, "%dLCG%d",   prec->axis, prec->lcg  );
    mmcCmd->send( cmd, clen );

    clen = sprintf( cmd, "%dLDR%d",   prec->axis, prec->ldr  );
    mmcCmd->send( cmd, clen );

    clen = sprintf( cmd, "%dLPL%d",   prec->axis, prec->lpl  );
    mmcCmd->send( cmd, clen );

    clen = sprintf( cmd, "%dMPL%d",   prec->axis, prec->mpl  );
    mmcCmd->send( cmd, clen );

    clen = sprintf( cmd, "%dREZ%d",   prec->axis, prec->rez  );
    mmcCmd->send( cmd, clen );

    clen = sprintf( cmd, "%dTLN%.6f", prec->axis, prec->dllm );
    mmcCmd->send( cmd, clen );

    clen = sprintf( cmd, "%dTLP%.6f", prec->axis, prec->dhlm );
    mmcCmd->send( cmd, clen );

    // closed loop deadband
    clen = sprintf( cmd, "%dDBD%d,%.3f", prec->axis, prec->dbd, prec->dbdt );
    mmcCmd->send( cmd, clen );

    clen = sprintf( cmd, "%dVMX?",    prec->axis             );
    mmcCmd->send( cmd, clen );

    if ( prec->dir == mmcaDIR_Positive )
    {
        prec->llm  = prec->off + prec->dllm;
        prec->hlm  = prec->off + prec->dhlm;
    }
    else
    {
        prec->llm  = prec->off - prec->dhlm;
        prec->hlm  = prec->off - prec->dllm;
    }

    // update the status
    clen = sprintf( cmd, "%dSTUP", prec->axis );
    mmcCmd->send( cmd, clen );

    prec->msta = msta.All;
    prec->mip  = MIP_DONE;
    prec->dmov = 1;

    prec->udf  = 1;

    mInfo->uMutex->unlock();

    log_msg( prec, 0, "Initializing, wait for status update" );

    recGblSetSevr( (dbCommon *)prec, COMM_ALARM, INVALID_ALARM );

    recGblGetTimeStamp( prec );

    alarm_mask = recGblResetAlarms( prec );
    post_fields( prec, alarm_mask, 1 );
    post_msgs  ( prec                );

    return( 0 );
}

/******************************************************************************/
static long process( dbCommon *precord )
{
    mmcaRecord      *prec  = (mmcaRecord *)precord;
    mmca_info       *mInfo = (mmca_info  *)prec->dpvt;
    char             cmd[MAX_MSG_SIZE];
    unsigned short   old_mip,  alarm_mask;
    short            old_movn, old_dmov, old_rcnt, old_miss, old_fbk;
    double           old_val,  old_dval, old_drbv, old_rbv, old_diff, diff;
    mmc_status       old_sta,  sta;
    motor_status     old_msta, msta;
    float            old_vrt;
    timespec         ts;
    int              clen;
    bool             first = FALSE;
    long             status = OK;

    if ( prec->pact ) return( OK );

    prec->pact = 1;
    log_msg( prec, 1, "Process" );

    mInfo->uMutex->lock();
    if      ( mInfo->newData ==  0 )                       // no new information
    {
        mInfo->uMutex->unlock();
        goto exit;
    }
    else if ( mInfo->newData <   0 )                          // serious problem
    {
        old_msta.All = msta.All = prec->msta;

        if      ( mInfo->newData == -2 ) msta.Bits.RA_PROBLEM  = 1;
        else if ( mInfo->newData == -1 ) msta.Bits.RA_COMM_ERR = 1;

        log_msg( prec, 0, mInfo->rsp );

        mInfo->init = -1;
        mInfo->uMutex->unlock();

        msta.Bits.NOT_INIT = 1;

        prec->udf  = 1;

        goto finished;
    }
    else if ( mInfo->newData ==  1 )                       // erroneous response
    {
        log_msg( prec, 0, mInfo->rsp );

        mInfo->newData = 0;
        mInfo->uMutex->unlock();

        post_msgs( prec );

        goto exit;
    }
    else if ( mInfo->newData ==  8 )                       // response to a read
    {
        update_field( mInfo );

        mInfo->newData = 0;
        mInfo->uMutex->unlock();

        post_msgs( prec );

        goto exit;
    }
    else if ( mInfo->newData !=  9 )                               // impossible
    {
        mInfo->newData = 0;
        mInfo->uMutex->unlock();

        goto exit;
    }

    // new status info
    old_sta.All  = prec->sta;
    old_msta.All = prec->msta;
    old_movn     = prec->movn;
    old_drbv     = prec->drbv;
    old_rbv      = prec->rbv ;
    old_vrt      = prec->vrt ;
    old_fbk      = prec->fbk ;

    prec->fbk            = mInfo->fbk; //make sure feedback setting is current to avoid confusion
    prec->sta  = sta.All = mInfo->sta;
    prec->vrt            = mInfo->vrt;

    prec->drbv = prec->fbk < mmcaFBK_Closed2 ? mInfo->tpos : mInfo->epos;
    prec->rbv  = prec->drbv * (1 - 2*prec->dir) + prec->off;

    if ( old_sta.All != prec->sta  ) MARK( M_STA  );
    if ( old_vrt     != prec->vrt  ) MARK( M_VRT  );
    if ( old_drbv    != prec->drbv ) MARK( M_DRBV );
    if ( old_rbv     != prec->rbv  ) MARK( M_RBV  );
    if ( old_fbk     != prec->fbk  ) MARK( M_FBK  );

    msta.All         = 0;

    msta.Bits.RA_ERR = sta.Bits.RA_ERR;

    if ( mInfo->init == -1 )
    {
        mInfo->init        = 1;

        msta.Bits.NOT_INIT = 0;
        log_msg( prec, 0, "Initialization completed" );

        first = TRUE;
    }

    mInfo->newData = 0;
    mInfo->uMutex->unlock();

    if ( sta.Bits.RA_STOPPED        ||                                // stopped
         ((! sta.Bits.RA_DEC ) &&
          (! sta.Bits.RA_VELO) &&
          (! sta.Bits.RA_ACC )    )    ) prec->movn = msta.Bits.RA_MOVING = 0;
    else                                 prec->movn = msta.Bits.RA_MOVING = 1;
 
    if ( old_movn != prec->movn ) MARK( M_MOVN );

    if ( (prec->movn || old_movn) && (prec->sds > 0) &&
         ((prec->mip & MIP_HOME) == 0) )
    {                                                         // check for stall
        clock_gettime( CLOCK_REALTIME, &ts );
        if ( old_movn == 0 )                                         // new move
            prec->msec = ts.tv_sec;
        else
        {
            if ( fabs(old_drbv - prec->drbv) > prec->velo * 0.01 )
                prec->msec = ts.tv_sec;
            else if ( labs(ts.tv_sec - prec->msec) > prec->sds )
            {
                msta.Bits.RA_STALL = 1;
                log_msg( prec, 0, "Stall detected" );

                if ( ! (prec->mip & MIP_STOP) )                   // should stop
                {
                    prec->mip  |= MIP_STOP;

                    clen = sprintf( cmd, "%dSTP", prec->axis );
                    mmcCmd->send( cmd, clen );
                }
            }
        }
    }

    if ( prec->movn ) goto finished;                             // still moving

    if ( prec->mip == MIP_DONE )             // was not moving, check slip_stall
    {
        old_diff   = prec->diff;
        prec->diff = prec->rbv - prec->val;

        if ( fabs(prec->diff) > prec->pdbd )
        {
            log_msg( prec, 0, "Slipped, diff = %.6g", prec->diff );
            msta.Bits.EA_SLIP_STALL = 1;
        }

        if ( old_diff != prec->diff ) MARK( M_DIFF );

        goto finished;
    }

    old_mip  = prec->mip ;
    old_dmov = prec->dmov;
    old_rcnt = prec->rcnt;
    old_miss = prec->miss;
    old_val  = prec->val ;
    old_dval = prec->dval;

    diff = prec->rbv - prec->val;

    if ( prec->mip & (MIP_JOG | MIP_STOP | MIP_PAUSE | MIP_HOME | MIP_M2L) )
    {                           // jogging, stop, pause, homing or move to limit
        short newval = 1;

        prec->rcnt = 0;

        if      ( prec->mip & MIP_JOG   )
        {
            prec->mip & MIP_STOP ?
            log_msg( prec, 0, "Stopped jogging at %.6g (DRBV: %.6g)", prec->rbv, prec->drbv ) :
            log_msg( prec, 0, "Jogging stopped at %.6g (DRBV: %.6g)", prec->rbv, prec->drbv ) ;
                                   
            prec->miss = 0;
        }
        else if ( prec->mip & MIP_STOP  )
        {
            log_msg( prec, 0, "Stopped at %.6g (DRBV: %.6g)",
                              prec->rbv, prec->drbv );

            prec->miss = 0;
        }
        else if ( prec->mip & MIP_PAUSE )
        {
            log_msg( prec, 0, "Paused at %.6g (DRBV: %.6g)",
                              prec->rbv, prec->drbv );

            newval = 0;
        }
        else if ( prec->mip & MIP_HOME  )
        {
            prec->miss         = 0;
            prec->athm         = 1;
            msta.Bits.RA_HOMED = 1;
            msta.Bits.RA_HOME  = 1;

            db_post_events( prec, &prec->athm, DBE_VAL_LOG );

            if ( prec->htyp == mmcHTYP_Limits )
                if ( prec->mip & MIP_HOMF )
                    log_msg( prec, 0, "Homed to HLS"          );
                else
                    log_msg( prec, 0, "Homed to LLS"          );
            else
                    log_msg( prec, 0, "Homed to encoder mark" );
        }
        else if ( prec->mip & MIP_MLP   )
            log_msg( prec, 0, "Reached positive limit" );
        else if ( prec->mip & MIP_MLN   )
            log_msg( prec, 0, "Reached negative limit" );

        if ( newval )
        {
            prec->mip  = MIP_DONE;
            prec->dmov = 1;
            prec->val  = prec->rbv;
            prec->dval = prec->drbv;
            prec->diff = 0;
        }
    }
    else if ( prec->mip & MIP_NEW ) new_move( prec );
    else if ( (fabs(diff) >= prec->rdbd) &&                 // not closed enough
              (prec->rtry > 0) && (prec->rcnt < prec->rtry) )  // can retry more
    {
        prec->mip  |= MIP_RETRY;

        log_msg( prec, 0, "Desired %.6g, reached %.6g, retrying %d ...",
                          prec->val,  prec->rbv,  ++prec->rcnt );

        do_move( prec );
    }
    else                             // close enough, or no (more) retry allowed
    {
        prec->diff = diff;
        if ( fabs(diff) < prec->rdbd )
        {
            log_msg( prec, 0, "Desired %.6g, reached %.6g",
                              prec->val, prec->rbv             );
            prec->miss = 0;
        }
        else
        {
            log_msg( prec, 0, "Desired %.6g, reached %.6g after %d retries",
                              prec->val, prec->rbv, prec->rcnt );
            prec->miss = 1;
        }

        prec->mip  = MIP_DONE;
        prec->dmov = 1;
        prec->rcnt = 0;
    }

    if ( old_mip  != prec->mip  ) MARK( M_MIP  );
    if ( old_dmov != prec->dmov ) MARK( M_DMOV );
    if ( old_rcnt != prec->rcnt ) MARK( M_RCNT );
    if ( old_miss != prec->miss ) MARK( M_MISS );
    if ( old_val  != prec->val  ) MARK( M_VAL  );
    if ( old_dval != prec->dval ) MARK( M_DVAL );

    finished:
    if      ( msta.Bits.RA_PROBLEM                        )          // software
    {
        recGblSetSevr( (dbCommon *)prec, STATE_ALARM, INVALID_ALARM );
    }
    else if ( msta.Bits.RA_COMM_ERR                       )          // hardware
    {
        recGblSetSevr( (dbCommon *)prec, COMM_ALARM,  INVALID_ALARM );
    }
    else if ( msta.Bits.NOT_INIT                          )          // NOT_INIT
    {
        recGblSetSevr( (dbCommon *)prec, STATE_ALARM, MAJOR_ALARM   );

        log_msg( prec, 0, "Please re-initialize" );
    }
    else if ( msta.Bits.RA_ERR || msta.Bits.RA_STALL || msta.Bits.EA_SLIP_STALL)
    {
        if ( msta.Bits.RA_ERR) log_msg( prec, 0, "Got error(s)" );

        recGblSetSevr( (dbCommon *)prec, STATE_ALARM, MINOR_ALARM   );
    }
    else if ( ! first )
    {
        if ( prec->sevr > NO_ALARM )                // had alarm/warnings before
            log_msg( prec, 0, "Alarm/warnings cleared" );
    }

    msta.Bits.RA_DONE = prec->dmov;

    prec->msta        = msta.All;
    if ( prec->msta != old_msta.All ) MARK( M_MSTA );

    recGblGetTimeStamp( prec );

    alarm_mask = recGblResetAlarms( prec );
    post_fields( prec, alarm_mask, 0 );
    post_msgs  ( prec                );

    recGblFwdLink     ( prec );                 // process the forward scan link

    exit:
    prec->proc = 0;
    prec->pact = 0;

    return( status );
}

/******************************************************************************/
static void new_move( mmcaRecord *prec )
{
    char  cmd[MAX_MSG_SIZE];
    int   clen, mdir;

    mdir = (prec->dir == mmcaDIR_Positive) ? 1 : -1;

    prec->dmov = 0;

    if      ( prec->mip & MIP_MLP )
    {
        log_msg( prec, 0, "Move to positive limit" );

        prec->mip  = MIP_MLP ;

        clen = mdir == 1 ? sprintf( cmd, "%dMLP", prec->axis ) : sprintf( cmd, "%dMLN", prec->axis );
        mmcCmd->send( cmd, clen );
    }
    else if ( prec->mip & MIP_MLN )
    {
        log_msg( prec, 0, "Move to negative limit" );

        prec->mip  = MIP_MLN ;

        clen = mdir == 1 ? sprintf( cmd, "%dMLN", prec->axis ) : sprintf( cmd, "%dMLP", prec->axis );
        mmcCmd->send( cmd, clen );
    }
    else
    {
        log_msg( prec, 0, "Move to %.6g (DVAL: %.6g)", prec->val, prec->dval );

        prec->mip  = MIP_MOVE;
        do_move( prec );
    }

    return;
}

/******************************************************************************/
static void do_move( mmcaRecord *prec )
{
    char  cmd[MAX_MSG_SIZE];
    int   clen;

    clen = sprintf( cmd, "%dMVA%f", prec->axis, prec->dval );
    mmcCmd->send( cmd, clen );

    return;
}

/******************************************************************************/
static long special( dbAddr *pDbAddr, int after )
{
    mmcaRecord      *prec = (mmcaRecord *)pDbAddr->precord;
    char             cmd[MAX_MSG_SIZE];
    short            old_dmov, old_rcnt, old_lvio;
    double           nval, old_val, old_dval, old_rbv, new_dval;
    unsigned short   old_mip, alarm_mask = 0;
    motor_status     msta;

    int              clen, mdir, fieldIndex = dbGetFieldIndex( pDbAddr );

    if ( after == 0 )
    {
        if      ( fieldIndex == mmcaRecordVAL  ) prec->oval = prec->val ;
        else if ( fieldIndex == mmcaRecordDVAL ) prec->oval = prec->dval;
        else if ( fieldIndex == mmcaRecordSPG  ) prec->oval = prec->spg ;
        else if ( fieldIndex == mmcaRecordDIR  ) prec->oval = prec->dir ;
        else if ( fieldIndex == mmcaRecordOFF  ) prec->oval = prec->off ;
        else if ( fieldIndex == mmcaRecordSET  ) prec->oval = prec->set ;
        else if ( fieldIndex == mmcaRecordLLM  ) prec->oval = prec->llm ;
        else if ( fieldIndex == mmcaRecordHLM  ) prec->oval = prec->hlm ;
        else if ( fieldIndex == mmcaRecordDLLM ) prec->oval = prec->dllm;
        else if ( fieldIndex == mmcaRecordDHLM ) prec->oval = prec->dhlm;
        else if ( fieldIndex == mmcaRecordAMAX ) prec->oval = prec->amax;
        else if ( fieldIndex == mmcaRecordACCL ) prec->oval = prec->accl;
        else if ( fieldIndex == mmcaRecordVELO ) prec->oval = prec->velo;
        else if ( fieldIndex == mmcaRecordJACC ) prec->oval = prec->jacc;
        else if ( fieldIndex == mmcaRecordJFRA ) prec->oval = prec->jfra;
        else if ( fieldIndex == mmcaRecordRDBD ) prec->oval = prec->rdbd;
        else if ( fieldIndex == mmcaRecordEAD  ) prec->oval = prec->ead ;
        else if ( fieldIndex == mmcaRecordEPL  ) prec->oval = prec->epl ;
        else if ( fieldIndex == mmcaRecordERES ) prec->oval = prec->eres;
        else if ( fieldIndex == mmcaRecordFBK  ) prec->oval = prec->fbk ;
        else if ( fieldIndex == mmcaRecordLCG  ) prec->oval = prec->lcg ;
        else if ( fieldIndex == mmcaRecordLDR  ) prec->oval = prec->ldr ;
        else if ( fieldIndex == mmcaRecordLPL  ) prec->oval = prec->lpl ;
        else if ( fieldIndex == mmcaRecordMPL  ) prec->oval = prec->mpl ;
        else if ( fieldIndex == mmcaRecordREZ  ) prec->oval = prec->rez ;
        else if ( fieldIndex == mmcaRecordDBD  ) prec->oval = prec->dbd ;
        else if ( fieldIndex == mmcaRecordDBDT ) prec->oval = prec->dbdt;

        return( OK );
    }

    old_val  = prec->val ;
    old_dval = prec->dval;
    old_rbv  = prec->rbv ;
    old_mip  = prec->mip ;
    old_dmov = prec->dmov;
    old_rcnt = prec->rcnt;
    old_lvio = prec->lvio;

    msta.All = prec->msta;

    switch( fieldIndex )
    {
        case mmcaRecordVAL :
            if ( (prec->sevr >  MINOR_ALARM) ||
                 ((prec->set == mmcaSET_Use) && (prec->spg != mmcaSPG_Go)) )
            {
                prec->val  = prec->oval;

                if      ( msta.Bits.RA_PROBLEM    )
                    log_msg( prec, 0, "No move/set, software problem"  );
                else if ( msta.Bits.RA_COMM_ERR   )
                    log_msg( prec, 0, "No move/set, hardware problem"  );
                else if ( msta.Bits.NOT_INIT      )
                    log_msg( prec, 0, "No move/set, init not finished" );
                else if ( prec->spg != mmcaSPG_Go )
                    log_msg( prec, 0, "No move, SPG is not Go"         );
                else
                    log_msg( prec, 0, "No move/set, unknown alarm"     );

                break;
            }

            new_dval = (prec->val - prec->off) * (1 - 2*prec->dir);
            if ( (prec->val < prec->llm) || (prec->val > prec->hlm) )
            {                                    // violated the software limits
                prec->lvio = 1;                     // set limit violation alarm
                prec->val  = prec->oval;

                log_msg( prec, 0, "No move/set, limit violated" );
                break;
            }

            do_move1:
            if ( prec->set == mmcaSET_Use )             // do it only when "Use"
                prec->dval = new_dval;

            goto do_move2;
        case mmcaRecordDVAL:
            if ( (prec->sevr >  MINOR_ALARM) ||
                 ((prec->set == mmcaSET_Use) && (prec->spg != mmcaSPG_Go)) )
            {
                prec->dval = prec->oval;

                if      ( msta.Bits.RA_PROBLEM    )
                    log_msg( prec, 0, "No move/set, software problem"  );
                else if ( msta.Bits.RA_COMM_ERR   )
                    log_msg( prec, 0, "No move/set, hardware problem"  );
                else if ( msta.Bits.NOT_INIT      )
                    log_msg( prec, 0, "No move/set, init not finished" );
                else if ( prec->spg != mmcaSPG_Go )
                    log_msg( prec, 0, "No move, SPG is not Go"         );
                else
                    log_msg( prec, 0, "No move/set, unknown alarm"     );

                break;
            }

            if ( (prec->dval < prec->dllm) || (prec->dval > prec->dhlm) )
            {                                    // violated the hardware limits
                prec->lvio = 1;                     // set limit violation alarm
                prec->dval = prec->oval;

                log_msg( prec, 0, "No move/set, limit violated" );
                break;
            }

            prec->val  = prec->dval * (1 - 2*prec->dir) + prec->off;

            do_move2:
            if ( prec->set == mmcaSET_Set ) break;                    // no move

            prec->lvio = 0;
            prec->rcnt = 0;

            prec->athm = 0;
            msta.Bits.RA_HOMED = 0;
            msta.Bits.RA_HOME  = 0;
            msta.Bits.EA_HOME  = 0;

            db_post_events( prec, &prec->athm, DBE_VAL_LOG );

            if ( prec->dmov == 0 )                               // still moving
            {
                if ( prec->mip != MIP_NEW )           // stop current move first
                {
                    log_msg( prec, 0, "Stop current move" );
                    prec->mip  = MIP_NEW;

                    clen = sprintf( cmd, "%dSTP", prec->axis );
                    mmcCmd->send( cmd, clen );
                }

                break;
            }

            new_move( prec );

            break;
        case mmcaRecordTWF :
            nval = prec->rbv + prec->twv;
            goto tweak;
        case mmcaRecordTWR :
            nval = prec->rbv - prec->twv;

            tweak:
            if ( (prec->sevr > MINOR_ALARM) || (prec->spg != mmcaSPG_Go) )
            {
                if      ( msta.Bits.RA_PROBLEM    )
                    log_msg( prec, 0, "No tweak, software problem"  );
                else if ( msta.Bits.RA_COMM_ERR   )
                    log_msg( prec, 0, "No tweak, hardware problem"  );
                else if ( msta.Bits.NOT_INIT      )
                    log_msg( prec, 0, "No tweak, init not finished" );
                else if ( prec->spg != mmcaSPG_Go )
                    log_msg( prec, 0, "No tweak, SPG is not Go"     );
                else
                    log_msg( prec, 0, "No tweak, unknown alarm"     );

                break;
            }

            new_dval = (nval - prec->off) * (1 - 2*prec->dir);
            if ( (nval < prec->llm) || (nval > prec->hlm) )
            {                                    // violated the software limits
                prec->lvio = 1;                     // set limit violation alarm
                log_msg( prec, 0, "No tweak, limit violated" );

                break;
            }

            prec->val = nval;
            goto do_move1;
        case mmcaRecordJOGF:
            if ( prec->jogf == 0 )
            {
                if ( prec->mip == MIP_JOGF )
                {
                    prec->mip |= MIP_STOP;

                    clen = sprintf( cmd, "%dSTP", prec->axis );
                    mmcCmd->send( cmd, clen );
                }

                break;
            }

            goto jogging_or_not;
        case mmcaRecordJOGR:
            if ( prec->jogr == 0 )
            {
                if ( prec->mip == MIP_JOGR )
                {
                    prec->mip |= MIP_STOP;

                    clen = sprintf( cmd, "%dSTP", prec->axis );
                    mmcCmd->send( cmd, clen );
                }

                break;
            }

            jogging_or_not:
            if ( (prec->sevr >  MINOR_ALARM) || (prec->mip != MIP_DONE) ||
                 (prec->spg  != mmcaSPG_Go )                               )
            {
                if      ( msta.Bits.RA_PROBLEM     )
                    log_msg( prec, 0, "No jogging, software problem"  );
                else if ( msta.Bits.RA_COMM_ERR    )
                    log_msg( prec, 0, "No jogging, hardware problem"  );
                else if ( msta.Bits.NOT_INIT       )
                    log_msg( prec, 0, "No jogging, init not finished" );
                else if ( prec->dmov != MIP_DONE   )
                    log_msg( prec, 0, "No jogging, motor is not idle" );
                else if ( prec->spg  != mmcaSPG_Go )
                    log_msg( prec, 0, "No jogging, SPG is not Go"     );
                else
                    log_msg( prec, 0, "No jogging, unknown alarm"     );

                if ( fieldIndex == mmcaRecordJOGF ) prec->jogf = 0;
                else                                prec->jogr = 0;

                break;
            }

            prec->lvio = 0;
            prec->rcnt = 0;

            prec->athm = 0;
            msta.Bits.RA_HOMED = 0;
            msta.Bits.RA_HOME  = 0;
            msta.Bits.EA_HOME  = 0;

            db_post_events( prec, &prec->athm, DBE_VAL_LOG );

            if ( fieldIndex == mmcaRecordJOGF ) prec->mip  = MIP_JOGF;
            else                                prec->mip  = MIP_JOGR;

            do_jog:
            prec->dmov = 0;

            mdir = (prec->dir == mmcaDIR_Positive) ? 1 : -1;
            if ( prec->mip == MIP_JOGF )
            {
                log_msg( prec, 0, "Jogging forward ..."  );

                clen = ( mdir == 1 ) ? sprintf( cmd, "%dJOG%.3f", prec->axis,  prec->jfra ) :
                                       sprintf( cmd, "%dJOG%.3f", prec->axis, -prec->jfra ) ;
            }
            else
            {
                log_msg( prec, 0, "Jogging backward ..." );

                clen = ( mdir == 1 ) ? sprintf( cmd, "%dJOG%.3f", prec->axis, -prec->jfra ) :
                                       sprintf( cmd, "%dJOG%.3f", prec->axis,  prec->jfra ) ;
            }

            mmcCmd->send( cmd, clen );

            break;
        case mmcaRecordMLP :
            prec->mlp  = 0;
        case mmcaRecordMLN :
            prec->mln  = 0;

            if ( (prec->sevr > MINOR_ALARM) || (prec->spg != mmcaSPG_Go) )
            {
                if      ( msta.Bits.RA_PROBLEM    )
                    log_msg( prec, 0, "No moving, software problem"  );
                else if ( msta.Bits.RA_COMM_ERR   )
                    log_msg( prec, 0, "No moving, hardware problem"  );
                else if ( msta.Bits.NOT_INIT      )
                    log_msg( prec, 0, "No moving, init not finished" );
                else if ( prec->spg != mmcaSPG_Go )
                    log_msg( prec, 0, "No moving, SPG is not Go"     );
                else
                    log_msg( prec, 0, "No moving, unknown alarm"     );

                break;
            }

            prec->lvio = 0;
            prec->rcnt = 0;

            prec->athm = 0;
            msta.Bits.RA_HOMED = 0;
            msta.Bits.RA_HOME  = 0;
            msta.Bits.EA_HOME  = 0;

            db_post_events( prec, &prec->athm, DBE_VAL_LOG );

            if ( fieldIndex == mmcaRecordMLP ) prec->mip  = MIP_MLP;
            else                               prec->mip  = MIP_MLN;

            if ( prec->dmov == 0 )                               // still moving
            {
                log_msg( prec, 0, "Stop current move" );
                prec->mip  |= MIP_NEW;

                clen = sprintf( cmd, "%dSTP", prec->axis );
                mmcCmd->send( cmd, clen );

                break;
            }

            new_move( prec );

            break;
        case mmcaRecordHOMF:
            prec->homf = 0;
        case mmcaRecordHOMR:
            prec->homr = 0;

            if ( prec->htyp == mmcHTYP_None )
            {
                log_msg( prec, 0, "Homing type not defined" );

                break;
            }

            if ( (prec->sevr > MINOR_ALARM) || (prec->spg != mmcaSPG_Go) )
            {
                if      ( msta.Bits.RA_PROBLEM    )
                    log_msg( prec, 0, "No homing, software problem"  );
                else if ( msta.Bits.RA_COMM_ERR   )
                    log_msg( prec, 0, "No homing, hardware problem"  );
                else if ( msta.Bits.NOT_INIT      )
                    log_msg( prec, 0, "No homing, init not finished" );
                else if ( prec->spg != mmcaSPG_Go )
                    log_msg( prec, 0, "No homing, SPG is not Go"     );
                else
                    log_msg( prec, 0, "No homing, unknown alarm"     );

                break;
            }

            if ( prec->dmov == 0 )                               // still moving
            {
                log_msg( prec, 0, "Please stop current move first" );

                break;
            }

            prec->lvio = 0;
            prec->rcnt = 0;

            prec->athm = 0;
            msta.Bits.RA_HOMED = 0;
            msta.Bits.RA_HOME  = 0;
            msta.Bits.EA_HOME  = 0;

            db_post_events( prec, &prec->athm, DBE_VAL_LOG );

            prec->mip = fieldIndex == mmcaRecordHOMF ? MIP_HOMF : MIP_HOMR;

            do_home:
            prec->dmov = 0;

            mdir = (prec->dir == mmcaDIR_Positive) ? 1 : -1;
            if ( prec->htyp == mmcHTYP_Limits )
            {
                if ( prec->mip == MIP_HOMF )
                {
                    log_msg( prec, 0, "Homing >> to HLS ..." );

                    clen = (mdir == 1) ? sprintf( cmd, "%dMLP", prec->axis ) :
                                         sprintf( cmd, "%dMLN", prec->axis ) ;
                }
                else
                {
                    log_msg( prec, 0, "Homing << to LLS ..." );

                    clen = (mdir == 1) ? sprintf( cmd, "%dMLN", prec->axis ) :
                                         sprintf( cmd, "%dMLP", prec->axis ) ;
                }
            }
            else
            {
                if ( prec->mip == MIP_HOMF )
                {
                    log_msg( prec, 0, "Homing >> to encoder mark ..." );

                    clen = (mdir == 1) ? sprintf( cmd, "%dHCG1", prec->axis ) :
                                         sprintf( cmd, "%dHCG0", prec->axis ) ;
                }
                else
                {
                    log_msg( prec, 0, "Homing << to encoder mark ..." );

                    clen = (mdir == 1) ? sprintf( cmd, "%dHCG0", prec->axis ) :
                                         sprintf( cmd, "%dHCG1", prec->axis ) ;
                }

                mmcCmd->send( cmd, clen );

                clen = sprintf( cmd, "%dHOM", prec->axis );
            }

            mmcCmd->send( cmd, clen );

            break;
        case mmcaRecordSPG :
            if ( (prec->spg == prec->oval) || (prec->mip == MIP_DONE) ) break;

            if ( prec->spg == mmcaSPG_Go )
            {
                if ( prec->mip & MIP_STOP ) break;

                prec->mip &= ~( MIP_STOP | MIP_PAUSE );
                if      ( prec->mip & MIP_JOG  ) goto do_jog;
                else if ( prec->mip & MIP_HOME ) goto do_home;
                else                             new_move( prec );

                break;
            }
            else
            {
                if ( prec->spg == mmcaSPG_Stop )
                {
                    log_msg( prec, 0, "Stop, with deceleration"  );
                    prec->mip &= ~MIP_PAUSE;
                    prec->mip |=  MIP_STOP ;
                }
                else
                {
                    log_msg( prec, 0, "Pause, with deceleration" );
                    prec->mip |=  MIP_PAUSE;
                }

                clen = sprintf( cmd, "%dSTP", prec->axis );
                mmcCmd->send( cmd, clen );
            }

            break;
        case ( mmcaRecordSTOP ):
            prec->stop = 0;
        case ( mmcaRecordESTP ):
            prec->estp = 0;

            if ( prec->mip == MIP_DONE ) break;

            prec->mip  &= ~MIP_PAUSE;
            prec->mip  |=  MIP_STOP ;
            if ( fieldIndex == mmcaRecordSTOP )
            {
                clen = sprintf( cmd, "%dSTP", prec->axis );

                log_msg( prec, 0, "Stop !!!" );
            }
            else
            {
                clen = sprintf( cmd, "%dEST", prec->axis );

                prec->spg = mmcaSPG_Stop;
                db_post_events( prec, &prec->spg,  DBE_VAL_LOG );

                log_msg( prec, 0, "Emergency stop !!!" );
            }

            mmcCmd->send( cmd, clen );

            break;
        case ( mmcaRecordDIR  ):
            if ( prec->dir == prec->oval ) break;

            if ( prec->dir == mmcaDIR_Positive )
            {
                prec->llm = prec->off + prec->dllm;
                prec->hlm = prec->off + prec->dhlm;
            }
            else
            {
                prec->llm = prec->off - prec->dhlm;
                prec->hlm = prec->off - prec->dllm;
            }

            nval       = prec->lls;
            prec->lls  = prec->hls;
            prec->hls  = nval;
            if ( prec->lls != nval )
            {
                db_post_events( prec, &prec->lls,  DBE_VAL_LOG );
                db_post_events( prec, &prec->hls,  DBE_VAL_LOG );
            }

            goto change_dir_off;
        case ( mmcaRecordOFF  ):
            prec->llm += prec->off - prec->oval;
            prec->hlm += prec->off - prec->oval;

            change_dir_off:
            db_post_events( prec, &prec->llm,  DBE_VAL_LOG );
            db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );

            prec->rbv  = prec->drbv * (1 - 2*prec->dir) + prec->off;
            prec->val  = prec->rbv;

//          check_software_limits( prec );

            goto stup;
        case mmcaRecordSET :
            if ( (prec->set == prec->oval ) ||
                 (prec->set == mmcaSET_Set)    ) break;

            prec->rbv  = prec->val;
            prec->off  = prec->rbv - prec->drbv * (1 - 2*prec->dir);
            db_post_events( prec, &prec->off,  DBE_VAL_LOG );

            if ( prec->dir == mmcaDIR_Positive )
            {
                prec->llm = prec->off + prec->dllm;
                prec->hlm = prec->off + prec->dhlm;
            }
            else
            {
                prec->llm = prec->off - prec->dhlm;
                prec->hlm = prec->off - prec->dllm;
            }

            db_post_events( prec, &prec->llm,  DBE_VAL_LOG );
            db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );

            break;
        case mmcaRecordLLM :
            if ( prec->llm > prec->hlm )            // illegal, retain old value
            {
                prec->llm  = prec->oval;
                db_post_events( prec, &prec->llm,  DBE_VAL_LOG );

                log_msg( prec, 0, "LLM > HLM, retain old LLM" );
                break;
            }

            if ( prec->dir == mmcaDIR_Positive )
            {
                nval = prec->llm - prec->off;
                if ( nval <= -1000 )
                {
                    prec->llm  = prec->oval;
                    db_post_events( prec, &prec->llm,  DBE_VAL_LOG );

                    log_msg( prec, 0, "DLLM <= -1000, retain old LLM" );
                    break;
                }

                prec->dllm = nval;
                db_post_events( prec, &prec->dllm, DBE_VAL_LOG );

                clen = sprintf( cmd, "%dTLN%.6f", prec->axis, prec->dllm );
                mmcCmd->send( cmd, clen );
            }
            else
            {
                nval = prec->off - prec->llm;
                if ( nval >=  1000 )
                {
                    prec->llm  = prec->oval;
                    db_post_events( prec, &prec->llm,  DBE_VAL_LOG );

                    log_msg( prec, 0, "DHLM >=  1000, retain old LLM" );
                    break;
                }

                prec->dhlm = nval;
                db_post_events( prec, &prec->dhlm, DBE_VAL_LOG );

                clen = sprintf( cmd, "%dTLP%.6f", prec->axis, prec->dhlm );
                mmcCmd->send( cmd, clen );
            }

            check_limit_violation:
            if ( (prec->val < prec->llm) || (prec->val > prec->hlm) )
                prec->lvio = 1;                   // set limit violation warning

            break;
        case mmcaRecordHLM :
            if ( prec->hlm < prec->llm )            // illegal, retain old value
            {
                prec->hlm  = prec->oval;
                db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );

                log_msg( prec, 0, "HLM < LLM, retain old HLM" );
                break;
            }

            if ( prec->dir == mmcaDIR_Positive )
            {
                nval = prec->hlm - prec->off;
                if ( nval >=  1000 )
                {
                    prec->hlm  = prec->oval;
                    db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );

                    log_msg( prec, 0, "DHLM >=  1000, retain old HLM" );
                    break;
                }

                prec->dhlm = nval;
                db_post_events( prec, &prec->dhlm, DBE_VAL_LOG );

                clen = sprintf( cmd, "%dTLP%.6f", prec->axis, prec->dhlm );
                mmcCmd->send( cmd, clen );
            }
            else
            {
                nval = prec->off - prec->hlm;
                if ( nval <= -1000 )
                {
                    prec->hlm  = prec->oval;
                    db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );

                    log_msg( prec, 0, "DLLM <= -1000, retain old HLM" );
                    break;
                }

                prec->dllm = nval;
                db_post_events( prec, &prec->dllm, DBE_VAL_LOG );

                clen = sprintf( cmd, "%dTLN%.6f", prec->axis, prec->dllm );
                mmcCmd->send( cmd, clen );
            }

            goto check_limit_violation;
        case mmcaRecordDLLM:
            if ( (prec->dllm <= -1000) || (prec->dllm > prec->dhlm) )
            {                                       // illegal, retain old value
                prec->dllm = prec->oval;
                db_post_events( prec, &prec->dllm, DBE_VAL_LOG );

                prec->dllm <= -1000 ? log_msg( prec, 0, "DLLM <= -1000, retain old DLLM" ) :
                                      log_msg( prec, 0, "DLLM > DHLM, retain old DLLM"   ) ;

                break;
            }

            if ( prec->dir == mmcaDIR_Positive )
            {
                prec->llm  = prec->off + prec->dllm;
                db_post_events( prec, &prec->llm,  DBE_VAL_LOG );
            }
            else
            {
                prec->hlm  = prec->off - prec->dllm;
                db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );
            }

            clen = sprintf( cmd, "%dTLN%.6f", prec->axis, prec->dllm );
            mmcCmd->send( cmd, clen );

            goto check_limit_violation;
        case mmcaRecordDHLM:
            if ( (prec->dhlm >=  1000) || (prec->dhlm < prec->dllm) )
            {                                       // illegal, retain old value
                prec->dhlm = prec->oval;
                db_post_events( prec, &prec->dhlm, DBE_VAL_LOG );

                if ( prec->dhlm >=  1000 )
                    log_msg( prec, 0, "DHLM >=  1000, retain old DHLM" );
                else
                    log_msg( prec, 0, "DHLM < DLLM, retain old DHLM"   );

                break;
            }

            if ( prec->dir == mmcaDIR_Positive )
            {
                prec->hlm  = prec->off + prec->dhlm;
                db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );
            }
            else
            {
                prec->llm  = prec->off - prec->dhlm;
                db_post_events( prec, &prec->llm,  DBE_VAL_LOG );
            }

            clen = sprintf( cmd, "%dTLP%.6f", prec->axis, prec->dhlm );
            mmcCmd->send( cmd, clen );

            goto check_limit_violation;
        case mmcaRecordAMAX:
            if ( prec->amax == prec->oval ) break;

            if ( prec->amax < prec->accl )
            {
                prec->accl = prec->amax;
                db_post_events( prec, &prec->accl, DBE_VAL_LOG );

                clen = sprintf( cmd, "%dACC%.3f", prec->axis, prec->accl );
                mmcCmd->send( cmd, clen );

                clen = sprintf( cmd, "%dDEC%.3f", prec->axis, prec->accl );
                mmcCmd->send( cmd, clen );

                log_msg( prec, 0, "Lowered ACCL to the new AMAX" );
            }

            clen = sprintf( cmd, "%dAMX%.3f", prec->axis, prec->amax );
            mmcCmd->send( cmd, clen );

            break;
        case mmcaRecordACCL:
            if ( prec->accl == prec->oval ) break;

            if ( prec->accl > prec->amax )
            {
                prec->accl = prec->oval;
                db_post_events( prec, &prec->accl, DBE_VAL_LOG );

                log_msg( prec, 0, "ACCL > AMAX, retain old value" );
            }
            else
            {
                clen = sprintf( cmd, "%dACC%.3f", prec->axis, prec->accl );
                mmcCmd->send( cmd, clen );

                clen = sprintf( cmd, "%dDEC%.3f", prec->axis, prec->accl );
                mmcCmd->send( cmd, clen );
            }

            break;
        case mmcaRecordVELO:
            if ( prec->velo == prec->oval ) break;

            if ( prec->velo > prec->vmax )
            {
                prec->velo = prec->oval;
                db_post_events( prec, &prec->velo, DBE_VAL_LOG );

                log_msg( prec, 0, "VELO > VMAX, retain old value" );
            }
            else
            {
                clen = sprintf( cmd, "%dVEL%.3f", prec->axis, prec->velo );
                mmcCmd->send( cmd, clen );
            }

            break;
        case mmcaRecordJACC:
            if ( prec->jacc == prec->oval ) break;

            if ( prec->jacc > prec->amax )
            {
                prec->jacc = prec->oval;
                db_post_events( prec, &prec->jacc, DBE_VAL_LOG );

                log_msg( prec, 0, "JACC > AMAX, retain old value" );
            }
            else
            {
                clen = sprintf( cmd, "%dJAC%.3f", prec->axis, prec->jacc );
                mmcCmd->send( cmd, clen );
            }

            break;
        case mmcaRecordJFRA:
            if ( prec->jfra == prec->oval ) break;

            if ( (prec->jfra < 0.001) || (prec->jfra > 100) )
            {
                prec->jfra = prec->oval;
                db_post_events( prec, &prec->jfra, DBE_VAL_LOG );

                log_msg( prec, 0, "JACC out of range (0.001 - 100), retain old value" );
            }

            break;
        case mmcaRecordEAD :
            if ( prec->ead  == prec->oval ) break;

            clen = sprintf( cmd, "%dEAD%d", prec->axis, prec->ead  );
            mmcCmd->send( cmd, clen );

            break;
        case mmcaRecordEPL :
            if ( prec->epl  == prec->oval ) break;

            clen = sprintf( cmd, "%dEPL%d", prec->axis, prec->epl  );
            mmcCmd->send( cmd, clen );

            break;
        case mmcaRecordERES:
            if ( prec->eres == prec->oval ) break;

            if ( prec->ead == mmcaEAD_Digital )
            {
                clen = sprintf( cmd, "%dENC%.3f", prec->axis, prec->eres*1000 );
                mmcCmd->send( cmd, clen );

                goto stup;
            }
            else
            {
                prec->eres = prec->oval;
                db_post_events( prec, &prec->eres, DBE_VAL_LOG );

                log_msg( prec, 0, "Cannot set resolution for analog encoder" );
            }

            break;
        case mmcaRecordFBK :
            if ( prec->fbk  == prec->oval ) break;

            clen = sprintf( cmd, "%dFBK%d", prec->axis, prec->fbk  );
            mmcCmd->send( cmd, clen );

            goto stup;
        case mmcaRecordLCG :
            if ( prec->lcg  == prec->oval ) break;

            clen = sprintf( cmd, "%dLCG%d", prec->axis, prec->lcg  );
            mmcCmd->send( cmd, clen );

            goto stup;
        case mmcaRecordLDR :
            if ( prec->ldr  == prec->oval ) break;

            clen = sprintf( cmd, "%dLDR%d", prec->axis, prec->ldr  );
            mmcCmd->send( cmd, clen );

            goto stup;
        case mmcaRecordLPL :
            if ( prec->lpl  == prec->oval ) break;

            clen = sprintf( cmd, "%dLPL%d", prec->axis, prec->lpl  );
            mmcCmd->send( cmd, clen );

            goto stup;
        case mmcaRecordMPL :
            if ( prec->mpl  == prec->oval ) break;

            clen = sprintf( cmd, "%dMPL%d", prec->axis, prec->mpl  );
            mmcCmd->send( cmd, clen );

            goto stup;
        case mmcaRecordREZ :
            if ( prec->rez  == prec->oval ) break;

            clen = sprintf( cmd, "%dREZ%d", prec->axis, prec->rez  );
            mmcCmd->send( cmd, clen );

            goto stup;
        case mmcaRecordDBD :
            if ( prec->dbd  == prec->oval ) break;

            goto dbdt;
        case mmcaRecordDBDT:
            if ( prec->dbdt == prec->oval ) break;

            dbdt:
            clen = sprintf( cmd, "%dDBD%d,%.3f", prec->axis, prec->dbd, prec->dbdt );
            mmcCmd->send( cmd, clen );

            break;
        case mmcaRecordSTUP:
            prec->stup = 0;

            stup:
            clen = sprintf( cmd, "%dSTUP",  prec->axis             );
            mmcCmd->send( cmd, clen );

            break;
        case mmcaRecordCMD :
            clen = sprintf( cmd, "%d%s",    prec->axis, prec->cmd  );
            mmcCmd->send( cmd, clen );

            break;
        case mmcaRecordRINI:
            init_axis( prec );

            prec->rini = 0;
            break;
    }

    if ( prec->val  != old_val  ) MARK( M_VAL  );
    if ( prec->dval != old_dval ) MARK( M_DVAL );
    if ( prec->rbv  != old_rbv  ) MARK( M_RBV  );
    if ( prec->mip  != old_mip  ) MARK( M_MIP  );
    if ( prec->dmov != old_dmov ) MARK( M_DMOV );
    if ( prec->rcnt != old_rcnt ) MARK( M_RCNT );
    if ( prec->lvio != old_lvio ) MARK( M_LVIO );

    post_fields( prec, alarm_mask, 0 );
    post_msgs  ( prec                );

    return( OK );
}

/******************************************************************************/
static long cvt_dbaddr( dbAddr *pDbAddr )
{
    mmcaRecord  *prec = (mmcaRecord *)pDbAddr->precord;
    mmca_info   *mInfo = (mmca_info *)prec->dpvt;

    int          fieldIndex = dbGetFieldIndex( pDbAddr );
    long         status = 0;

    switch ( fieldIndex )
    {
        case mmcaRecordLOGA:
        {
            pDbAddr->pfield         = (char *)prec->loga;
            pDbAddr->no_elements    = mInfo->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case mmcaRecordLOGB:
        {
            pDbAddr->pfield         = (char *)prec->logb;
            pDbAddr->no_elements    = mInfo->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case mmcaRecordLOGC:
        {
            pDbAddr->pfield         = (char *)prec->logc;
            pDbAddr->no_elements    = mInfo->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case mmcaRecordLOGD:
        {
            pDbAddr->pfield         = (char *)prec->logd;
            pDbAddr->no_elements    = mInfo->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case mmcaRecordLOGE:
        {
            pDbAddr->pfield         = (char *)prec->loge;
            pDbAddr->no_elements    = mInfo->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case mmcaRecordLOGF:
        {
            pDbAddr->pfield         = (char *)prec->logf;
            pDbAddr->no_elements    = mInfo->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case mmcaRecordLOGG:
        {
            pDbAddr->pfield         = (char *)prec->logg;
            pDbAddr->no_elements    = mInfo->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case mmcaRecordLOGH:
        {
            pDbAddr->pfield         = (char *)prec->logh;
            pDbAddr->no_elements    = mInfo->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
    }

    return( status );
}

/******************************************************************************/
static long get_precision( dbAddr *pDbAddr, long *precision )
{
    mmcaRecord *prec = (mmcaRecord *)pDbAddr->precord;
    int fieldIndex = dbGetFieldIndex( pDbAddr );

    switch ( fieldIndex )
    {
        case mmcaRecordVERS:
            *precision = 2;

            break;
        case mmcaRecordERES:
            *precision = 6;

            break;
        case mmcaRecordDBDT:
            *precision = 3;

            break;
        case mmcaRecordAMAX:
            *precision = 3;

            break;
        case mmcaRecordACCL:
            *precision = 3;

            break;
        case mmcaRecordJACC:
            *precision = 3;

            break;
        case mmcaRecordVMAX:
            *precision = 3;

            break;
        case mmcaRecordVELO:
            *precision = 3;

            break;
        case mmcaRecordVRT :
            *precision = 3;

            break;
        case mmcaRecordJFRA:
            *precision = 3;

            break;
        default:
            *precision = prec->prec;

            break;
    }

    return( 0 );
}

/******************************************************************************/
static long update_field( struct mmca_info *mInfo )
{
    mmcaRecord  *prec = mInfo->precord;
    int          axis, coff;
    float        eres, vmax;
    long         status;

    coff = prec->axis > 9 ? 2 : 1;

    strcpy( prec->resp, mInfo->rsp );
    db_post_events( prec,  prec->resp, DBE_VAL_LOG );

    if      ( strncmp(mInfo->rsp+coff, "VER", 3) == 0 )
    {
        strcpy( prec->fver, mInfo->rsp+coff+4 );
        db_post_events( prec,  prec->fver, DBE_VAL_LOG );
    }
    else if ( strncmp(mInfo->rsp+coff, "ENC", 3) == 0 )
    {
        status = sscanf( mInfo->rsp, "%dENC#%f", &axis, &eres );
        if ( status == 2 )
        {
            prec->eres = eres / 1000;
            db_post_events( prec, &prec->eres, DBE_VAL_LOG );
        }
    }
    else if ( strncmp(mInfo->rsp+coff, "VMX", 3) == 0 )
    {
        status = sscanf( mInfo->rsp, "%dVMX#%f", &axis, &vmax );
        if ( status == 2 )
        {
            prec->vmax = vmax;
            db_post_events( prec, &prec->vmax, DBE_VAL_LOG );
        }
    }
    else if ( strncmp(mInfo->rsp+coff, "ERR", 3) == 0 )
        log_msg( prec, 0, mInfo->rsp+coff+4 );

    return( 0 );
}

/******************************************************************************/
static void post_fields( mmcaRecord *prec, unsigned short alarm_mask,
                                           unsigned short all )
{
    unsigned short  field_mask;
    changed_fields  cmap;

    cmap.All = prec->cmap;

    if ( (field_mask = alarm_mask | (all                  ? DBE_VAL_LOG : 0)) )
    {
        db_post_events( prec, &prec->vers, field_mask );
        db_post_events( prec,  prec->desc, field_mask );
        db_post_events( prec,  prec->fver, field_mask );
        db_post_events( prec,  prec->egu,  field_mask );
        db_post_events( prec, &prec->eres, field_mask );
        db_post_events( prec, &prec->dllm, field_mask );
        db_post_events( prec, &prec->dhlm, field_mask );
        db_post_events( prec, &prec->llm,  field_mask );
        db_post_events( prec, &prec->hlm,  field_mask );
    }

    if ( (field_mask = alarm_mask | (all | MARKED(M_STA ) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->sta,  field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_VRT ) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->vrt,  field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_VAL ) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->val,  field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_DVAL) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->dval, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_DRBV) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->drbv, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_RBV ) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->rbv,  field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_DIFF) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->diff, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_MIP ) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->mip,  field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_MOVN) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->movn, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_DMOV) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->dmov, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_RCNT) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->rcnt, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_MISS) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->miss, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_LVIO) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->lvio, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_RLLS) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->rlls, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_RHLS) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->rhls, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_LLS ) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->lls,  field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_HLS ) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->hls,  field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_MSTA) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->msta, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_FBK) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->fbk, field_mask );

    UNMARK_ALL;

    return;
}

/******************************************************************************/
static void getResponse( struct mmca_info *mInfo )
{
    mmcaRecord  *prec = mInfo->precord;
    char         rsp[MAX_MSG_SIZE];
    int          mlen, axis, sta, nread, fbk;
    float        vrt, tpos, epos;

    while ( ! interruptAccept ) epicsThreadSleep( 1 );

    while ( (mlen = mmcRsp[prec->axis-1]->receive(rsp, MAX_MSG_SIZE)) > 0 )
    {                                                          // got a response
        rsp[mlen] = '\0';

        mInfo->uMutex->lock();

        if ( strcmp(rsp, "Failed to communicate with the controller") == 0 )
        {
            strncpy( mInfo->rsp, rsp, MAX_MSG_SIZE );
            mInfo->newData = -1;
        }
        else
        {
            nread = sscanf( rsp, "%dSTA#%d,VRT#%f,POS#%f,%f,FBK#%d",
                                 &axis, &sta, &vrt, &tpos, &epos, &fbk );
            if ( nread == 6 )
            {
                if ( axis == prec->axis )
                {
                    mInfo->sta     = sta;
                    mInfo->vrt     = vrt;
                    mInfo->tpos    = tpos;
                    mInfo->epos    = epos;
                    mInfo->fbk     = fbk;

                    mInfo->newData = 9;
                }
                else
                {
                    strncpy( mInfo->rsp, rsp, MAX_MSG_SIZE );
                    mInfo->newData = 1;
                }
            }
            else
            {
                strncpy( mInfo->rsp, rsp, MAX_MSG_SIZE );
                mInfo->newData = 8;
            }
        }

        mInfo->uMutex->unlock();

        dbScanLock  ( (dbCommon *)prec );
        process     ( (dbCommon *)prec );
        dbScanUnlock( (dbCommon *)prec );
    }

    mInfo->uMutex->lock();

    strcpy( mInfo->rsp, "Listening thread exited" );
    mInfo->newData = -2;

    mInfo->uMutex->unlock();

    dbScanLock  ( (dbCommon *)prec );
    process     ( (dbCommon *)prec );
    dbScanUnlock( (dbCommon *)prec );
}

/******************************************************************************/
static long log_msg( mmcaRecord *prec, int dlvl, const char *fmt, ... )
{
    mmca_info  *mInfo = (mmca_info *)prec->dpvt;
    timespec    ts;
    struct tm   timeinfo;
    char        timestamp[40], msec[4], msg[512];

    va_list     args;

    if ( dlvl > max((int)prec->dlvl, (int)mmcaRecordDebug) ) return( 0 );

    clock_gettime( CLOCK_REALTIME, &ts );
    localtime_r( &ts.tv_sec, &timeinfo );

    strftime( timestamp, 40, "%m/%d %H:%M:%S", &timeinfo );
    sprintf ( msec, "%03d", int(ts.tv_nsec*1.e-6 + 0.5) );

    va_start( args, fmt      );
    vsprintf( msg, fmt, args );
    va_end  ( args           );

    if ( dlvl <= prec->dlvl )
    {
        mInfo->lMutex->lock();

        if ( mInfo->cIndex > 0 )
        {
            if ( strncmp(mInfo->sAddr+mInfo->mLength*(mInfo->cIndex-1)+9,
                         msg, strlen(msg)                                )==0 )
                mInfo->cIndex--;
            else if ( mInfo->cIndex > 7 )
                memmove( mInfo->sAddr,
                         mInfo->sAddr+mInfo->mLength, mInfo->mLength*7 );
        }

        snprintf( mInfo->sAddr+mInfo->mLength*min(mInfo->cIndex,7), 61,
                  "%s %s", timestamp+6, msg );

        if ( mInfo->cIndex <= 7 ) mInfo->cIndex++;

        mInfo->newMsg = 1;

        mInfo->lMutex->unlock();
    }

    if ( dlvl <= mmcaRecordDebug )
        printf( "%s.%s %s -- %s\n", timestamp, msec, prec->name, msg );

    return( 1 );
}

/******************************************************************************/
static void post_msgs( mmcaRecord *prec )
{
    mmca_info  *mInfo = (mmca_info *)prec->dpvt;

    mInfo->lMutex->lock();

    if ( mInfo->newMsg == 0 )
    {
        mInfo->lMutex->unlock();
        return;
    }

    db_post_events( prec,  prec->loga, DBE_VAL_LOG );
    db_post_events( prec,  prec->logb, DBE_VAL_LOG );
    db_post_events( prec,  prec->logc, DBE_VAL_LOG );
    db_post_events( prec,  prec->logd, DBE_VAL_LOG );
    db_post_events( prec,  prec->loge, DBE_VAL_LOG );
    db_post_events( prec,  prec->logf, DBE_VAL_LOG );
    db_post_events( prec,  prec->logg, DBE_VAL_LOG );
    db_post_events( prec,  prec->logh, DBE_VAL_LOG );

    mInfo->newMsg = 0;
    mInfo->lMutex->unlock();

    return;
}

