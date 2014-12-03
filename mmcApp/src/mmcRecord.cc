#define VERSION 1.0

#include <algorithm>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <list.h>
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
#include "mmcRecord.h"
#undef GEN_SIZE_OFFSET

#include "drvAsynIPPort.h"
#include "asynDriver.h"
#include "asynOctet.h"
#include "asynOctetSyncIO.h"

#include "epicsMessageQueue.h"
#include "epicsExport.h"

#include "mmc.h"


/*** Forward references ***/


/*** Record Support Entry Table (RSET) functions ***/

static long init_record( dbCommon *precord, int pass );
static long process    ( dbCommon *precord           );


rset mmcRSET =
{
    RSETNUMBER,
    NULL,
    NULL,
    (RECSUPFUN) init_record,
    (RECSUPFUN) process,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL
};

extern "C" { epicsExportAddress( rset, mmcRSET ); }


static long send_n_recv( asynUser *pasynUser, int axis, char *cmd, char *rsp  );
static long read_status( asynUser *pasynUser, int axis                        );
static void handle_cmd ( asynUser *pasynUser, char *cmd, list<int> &moving_new);
static void serial_io  ( asynUser *pasynUser                                  );


epicsMessageQueue *mmcCmd;
epicsMessageQueue *mmcRsp[99];


/*** Debugging ***/

volatile int mmcRecordDebug = 0;

extern "C" { epicsExportAddress( int, mmcRecordDebug ); }

using namespace std;


/******************************************************************************/
static long init_record( dbCommon *precord, int pass )
{
    mmcRecord         *prec = (mmcRecord *)precord;

    char               serialPort[80];
    asynUser          *pasynUser;
    static const char  output_terminator[] = "\n\r";
    static const char  input_terminator [] = "\n\r";

    asynStatus         asyn_rtn;
    long               status = 0;

    if ( pass > 0 ) return( status );

    mmcCmd         = new epicsMessageQueue( prec->naxs*100, MAX_MSG_SIZE );
    for ( int im = 0; im < prec->naxs; im++ )
        mmcRsp[im] = new epicsMessageQueue( 100,            MAX_MSG_SIZE );

    sprintf( serialPort, "%s TCP", prec->port );

    drvAsynIPPortConfigure              ( prec->asyn, serialPort, 0, 0, 0 );
    asyn_rtn = pasynOctetSyncIO->connect( prec->asyn, 0, &pasynUser, NULL );

    if ( asyn_rtn != asynSuccess )
        printf( "Failed to connect to port %s\n", prec->port );

    pasynOctetSyncIO->setOutputEos( pasynUser, output_terminator,
                                    strlen(output_terminator)     );
    pasynOctetSyncIO->setInputEos ( pasynUser, input_terminator,
                                    strlen(input_terminator)      );

    // flush any junk at input port - should not be any data yet
    pasynOctetSyncIO->flush( pasynUser );

    epicsThreadCreate( prec->name, epicsThreadPriorityMedium,
                       epicsThreadGetStackSize(epicsThreadStackMedium),
                       (EPICSTHREADFUNC)serial_io, (void *)pasynUser );

    return( status );
}

/******************************************************************************/
static long process( dbCommon *precord )
{
    return( 0 );
}

/******************************************************************************/
static long send_n_recv( asynUser *pasynUser, int axis, char *cmd, char *rsp )
{
    const double  timeout = 1.0;
    size_t        nwrite, nread = 0;
    int           it, coff, eomReason;
    asynStatus    asyn_rtn = asynError;

    pasynOctetSyncIO->flush( pasynUser );
    pasynOctetSyncIO->write( pasynUser, cmd, strlen(cmd), timeout, &nwrite );

    if ( axis > 9 ) coff = 2;
    else            coff = 1;

    if ( (strncmp(cmd+coff, "RST", 3) == 0) || (cmd[strlen(cmd)-1] == '?') )
    {                                                            // need to read
        for ( it = 0; it < 60; it++ )
        {
            nread = 0;
            asyn_rtn = pasynOctetSyncIO->read( pasynUser, rsp, MAX_MSG_SIZE,
                                               timeout, &nread, &eomReason );
            if ( (asyn_rtn == asynSuccess) && (nread > 0) )
                break;
            else
            {
                rsp[0] = '\0';
                nread  = 0;
            }
        }

        if ( nread <= 0 ) printf( "Failed to read controller #%d\n", axis );

        if ( cmd[strlen(cmd)-1] != '?' )
        {
            rsp[0] = '\0';
            nread  = 0;
        }
    }

    epicsThreadSleep( 0.05 );

    return( nread );
}

/******************************************************************************/
static long read_status( asynUser *pasynUser, int axis )   // return status byte
{
    char  cmd [MAX_MSG_SIZE];
    char  rsp1[MAX_MSG_SIZE], rsp2[MAX_MSG_SIZE], rsp3[MAX_MSG_SIZE];
    int   mlen, sta;
    long  status = 0;

    // read the status byte
    mlen   = sprintf( cmd, "%dSTA?", axis );
    status = send_n_recv( pasynUser, axis, cmd, rsp1 );

    // read the encoder velocity
    mlen   = sprintf( cmd, "%dVRT?", axis );
    status = send_n_recv( pasynUser, axis, cmd, rsp2 );

    // read the position
    mlen   = sprintf( cmd, "%dPOS?", axis );
    status = send_n_recv( pasynUser, axis, cmd, rsp3 );

    status = sscanf( rsp1, "#%d", &sta );

    // send info to the axis record
    if ( (strlen(rsp1) > 0) && (strlen(rsp2) > 0) && (strlen(rsp3) > 0) )
    {
        mlen = sprintf( cmd, "%dSTA%s,VRT%s,POS%s", axis, rsp1, rsp2, rsp3 );
        mmcRsp[axis-1]->send( cmd, mlen );
    }

    return( sta );
}

/******************************************************************************/
static void handle_cmd( asynUser *pasynUser, char *cmd, list<int> &moving_new )
{
    char  rsp[MAX_MSG_SIZE], rpl[MAX_MSG_SIZE];
    int   axis, coff, mlen;
    long  status = OK;

    status = sscanf( cmd, "%d", &axis );
    if ( (status == 1) && (axis >= 1) && (axis <= 99) )
    {
        if ( axis > 9 ) coff = 2;
        else            coff = 1;

        if ( strncmp(cmd+coff, "STUP", 4) == 0 )
            status = read_status( pasynUser, axis );
        else
        {
            status = send_n_recv( pasynUser, axis, cmd, rsp );
            if ( (cmd[strlen(cmd)-1] == '?') && (strlen(rsp) > 0) )
            {                                          // send back the response
                strncpy( rpl, cmd, strlen(cmd)-1 );
                sprintf( rpl+strlen(cmd)-1, "%s", rsp );
                mmcRsp[axis-1]->send( rpl, strlen(rpl) );
            }

            if ( (strncmp(cmd+coff, "MV",  2) == 0) ||
                 (strncmp(cmd+coff, "ML",  2) == 0) ||
                 (strncmp(cmd+coff, "JOG", 3) == 0) ||
                 (strncmp(cmd+coff, "HOM", 3) == 0)    )
                moving_new.push_back( axis );
        }
    }
    else 
        printf( "Bad command\n" );

    return;
}

/******************************************************************************/
static void serial_io( asynUser *pasynUser )
{
    char                 cmd[MAX_MSG_SIZE];
    list<int>            moving, moving_new;
    list<int>::iterator  ia;
    int                  mlen;
    mmc_status           sta;
    long                 status = OK;

    while ( ! interruptAccept ) epicsThreadSleep( 1 );

    while ( 1 )
    {
        if ( moving.size() == 0 )                           // no axis is moving
        {
            while ( (mlen = mmcCmd->tryReceive(cmd, MAX_MSG_SIZE)) > 0 )
            {
                cmd[mlen] = '\0';
                handle_cmd( pasynUser, cmd, moving_new );
            }

            if ( moving_new.size() == 0 )                        // no new moves
            {
                mlen = mmcCmd->receive( cmd, MAX_MSG_SIZE );
                if ( mlen > 0 )
                {
                    cmd[mlen] = '\0';
                    handle_cmd( pasynUser, cmd, moving_new );
                }
                else 
                    printf( "Bad command\n" );
            }
        }
        else
        {
            ia = moving.begin();
            while ( ia != moving.end() )
            {
                // read the status
                sta.All = read_status( pasynUser, *ia );
                if ( sta.Bits.RA_STOPPED ) ia = moving.erase( ia );   // stopped
                else                       ia++;

                while ( (mlen = mmcCmd->tryReceive(cmd, MAX_MSG_SIZE)) > 0 )
                {
                    cmd[mlen] = '\0';
                    handle_cmd( pasynUser, cmd, moving_new );
                }
            }
        }

        // merge old and new moving lists
        moving.splice( moving.begin(), moving_new );
    }
}

