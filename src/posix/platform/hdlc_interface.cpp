/*
 *  Copyright (c) 2018, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file includes the implementation for the HDLC interface to radio (RCP).
 */

#include "hdlc_interface.hpp"

#include "platform-posix.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#if OPENTHREAD_POSIX_CONFIG_RCP_PTY_ENABLE
#if defined(__APPLE__) || defined(__NetBSD__)
#include <util.h>
#elif defined(__FreeBSD__)
#include <libutil.h>
#else
#include <pty.h>
#endif
#endif
#include <stdarg.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <syslog.h>
#include <termios.h>
#include <unistd.h>

#include <openthread/logging.h>

#include "common/code_utils.hpp"
#include "lib/spinel/spinel.h"

#ifdef __APPLE__

#ifndef B230400
#define B230400 230400
#endif

#ifndef B460800
#define B460800 460800
#endif

#ifndef B500000
#define B500000 500000
#endif

#ifndef B576000
#define B576000 576000
#endif

#ifndef B921600
#define B921600 921600
#endif

#ifndef B1000000
#define B1000000 1000000
#endif

#ifndef B1152000
#define B1152000 1152000
#endif

#ifndef B1500000
#define B1500000 1500000
#endif

#ifndef B2000000
#define B2000000 2000000
#endif

#ifndef B2500000
#define B2500000 2500000
#endif

#ifndef B3000000
#define B3000000 3000000
#endif

#ifndef B3500000
#define B3500000 3500000
#endif

#ifndef B4000000
#define B4000000 4000000
#endif

#ifndef IOSSIOSPEED
#define IOSSIOSPEED 0x80045402
#endif

#endif // __APPLE__

#if OPENTHREAD_POSIX_CONFIG_SPINEL_HDLC_INTERFACE_ENABLE

namespace ot {
namespace Posix {

const char HdlcInterface::kLogModuleName[] = "HdlcIntface";

HdlcInterface::HdlcInterface(const Url::Url &aRadioUrl)
    : mReceiveFrameCallback(nullptr)
    , mReceiveFrameContext(nullptr)
    , mReceiveFrameBuffer(nullptr)
    , mSockFd(-1)
    , mBaudRate(0)
    , mHdlcDecoder()
    , mRadioUrl(aRadioUrl)
{
    memset(&mInterfaceMetrics, 0, sizeof(mInterfaceMetrics));
    mInterfaceMetrics.mRcpInterfaceType = kSpinelInterfaceTypeHdlc;
}

otError HdlcInterface::Init(ReceiveFrameCallback aCallback, void *aCallbackContext, RxFrameBuffer &aFrameBuffer)
{
    otError     error = OT_ERROR_NONE;
    struct stat st;

    LogCrit("@@@ 1");
    VerifyOrExit(mSockFd == -1, error = OT_ERROR_ALREADY);

    LogCrit("@@@ 2");
    VerifyOrDie(stat(mRadioUrl.GetPath(), &st) == 0, OT_EXIT_ERROR_ERRNO);

    LogCrit("@@@ 3");
    if (S_ISCHR(st.st_mode))
    {
        LogCrit("@@@ 4");
        mSockFd = OpenFile(mRadioUrl);
        LogCrit("@@@ 5");
        VerifyOrExit(mSockFd != -1, error = OT_ERROR_FAILED);
        LogCrit("@@@ 6");
    }
#if OPENTHREAD_POSIX_CONFIG_RCP_PTY_ENABLE
    else if (S_ISREG(st.st_mode))
    {
        LogCrit("@@@ 7");
        mSockFd = ForkPty(mRadioUrl);
        LogCrit("@@@ 8");
        VerifyOrExit(mSockFd != -1, error = OT_ERROR_FAILED);
        LogCrit("@@@ 9");
    }
#endif // OPENTHREAD_POSIX_CONFIG_RCP_PTY_ENABLE
    else
    {
        LogCrit("@@@ 10");
        LogCrit("Radio file '%s' not supported", mRadioUrl.GetPath());
        LogCrit("@@@ 11");
        ExitNow(error = OT_ERROR_FAILED);
        LogCrit("@@@ 12");
    }

    LogCrit("@@@ 13");
    mHdlcDecoder.Init(aFrameBuffer, HandleHdlcFrame, this);
    LogCrit("@@@ 14");
    mReceiveFrameCallback = aCallback;
    mReceiveFrameContext  = aCallbackContext;
    mReceiveFrameBuffer   = &aFrameBuffer;

exit:
    return error;
}

HdlcInterface::~HdlcInterface(void) { Deinit(); }

void HdlcInterface::Deinit(void)
{
    CloseFile();

    mReceiveFrameCallback = nullptr;
    mReceiveFrameContext  = nullptr;
    mReceiveFrameBuffer   = nullptr;
}

void HdlcInterface::Read(void)
{
    uint8_t buffer[kMaxFrameSize];
    ssize_t rval;

    rval = read(mSockFd, buffer, sizeof(buffer));

    if (rval > 0)
    {
        Decode(buffer, static_cast<uint16_t>(rval));
    }
    else if ((rval < 0) && (errno != EAGAIN) && (errno != EINTR))
    {
        DieNow(OT_EXIT_ERROR_ERRNO);
    }
}

void HdlcInterface::Decode(const uint8_t *aBuffer, uint16_t aLength) { mHdlcDecoder.Decode(aBuffer, aLength); }

otError HdlcInterface::SendFrame(const uint8_t *aFrame, uint16_t aLength)
{
    otError                            error = OT_ERROR_NONE;
    Spinel::FrameBuffer<kMaxFrameSize> encoderBuffer;
    Hdlc::Encoder                      hdlcEncoder(encoderBuffer);

    SuccessOrExit(error = hdlcEncoder.BeginFrame());
    SuccessOrExit(error = hdlcEncoder.Encode(aFrame, aLength));
    SuccessOrExit(error = hdlcEncoder.EndFrame());

    error = Write(encoderBuffer.GetFrame(), encoderBuffer.GetLength());

exit:
    if ((error == OT_ERROR_NONE) && IsSpinelResetCommand(aFrame, aLength))
    {
        mHdlcDecoder.Reset();
        error = ResetConnection();
    }

    return error;
}

otError HdlcInterface::Write(const uint8_t *aFrame, uint16_t aLength)
{
    otError error = OT_ERROR_NONE;
#if OPENTHREAD_POSIX_VIRTUAL_TIME
    virtualTimeSendRadioSpinelWriteEvent(aFrame, aLength);
#else
    while (aLength)
    {
        ssize_t rval = write(mSockFd, aFrame, aLength);

        if (rval == aLength)
        {
            break;
        }
        else if (rval > 0)
        {
            aLength -= static_cast<uint16_t>(rval);
            aFrame += static_cast<uint16_t>(rval);
        }
        else if (rval < 0)
        {
            VerifyOrDie((errno == EAGAIN) || (errno == EWOULDBLOCK) || (errno == EINTR), OT_EXIT_ERROR_ERRNO);
        }

        SuccessOrExit(error = WaitForWritable());
    }

exit:
#endif // OPENTHREAD_POSIX_VIRTUAL_TIME

    mInterfaceMetrics.mTransferredFrameCount++;
    if (error == OT_ERROR_NONE)
    {
        mInterfaceMetrics.mTxFrameCount++;
        mInterfaceMetrics.mTxFrameByteCount += aLength;
        mInterfaceMetrics.mTransferredValidFrameCount++;
    }
    else
    {
        mInterfaceMetrics.mTransferredGarbageFrameCount++;
    }

    return error;
}

otError HdlcInterface::WaitForFrame(uint64_t aTimeoutUs)
{
    otError        error = OT_ERROR_NONE;
    struct timeval timeout;
#if OPENTHREAD_POSIX_VIRTUAL_TIME
    struct VirtualTimeEvent event;

    timeout.tv_sec  = static_cast<time_t>(aTimeoutUs / OT_US_PER_S);
    timeout.tv_usec = static_cast<suseconds_t>(aTimeoutUs % OT_US_PER_S);

    virtualTimeSendSleepEvent(&timeout);
    virtualTimeReceiveEvent(&event);

    switch (event.mEvent)
    {
    case OT_SIM_EVENT_RADIO_SPINEL_WRITE:
        Decode(event.mData, event.mDataLength);
        break;

    case OT_SIM_EVENT_ALARM_FIRED:
        VerifyOrExit(event.mDelay <= aTimeoutUs, error = OT_ERROR_RESPONSE_TIMEOUT);
        break;

    default:
        assert(false);
        break;
    }
#else  // OPENTHREAD_POSIX_VIRTUAL_TIME
    timeout.tv_sec = static_cast<time_t>(aTimeoutUs / OT_US_PER_S);
    timeout.tv_usec = static_cast<suseconds_t>(aTimeoutUs % OT_US_PER_S);

    fd_set read_fds;
    fd_set error_fds;
    int rval;

    FD_ZERO(&read_fds);
    FD_ZERO(&error_fds);
    FD_SET(mSockFd, &read_fds);
    FD_SET(mSockFd, &error_fds);

    rval = select(mSockFd + 1, &read_fds, nullptr, &error_fds, &timeout);

    if (rval > 0)
    {
        if (FD_ISSET(mSockFd, &read_fds))
        {
            Read();
        }
        else if (FD_ISSET(mSockFd, &error_fds))
        {
            DieNowWithMessage("NCP error", OT_EXIT_FAILURE);
        }
        else
        {
            DieNow(OT_EXIT_FAILURE);
        }
    }
    else if (rval == 0)
    {
        ExitNow(error = OT_ERROR_RESPONSE_TIMEOUT);
    }
    else if (errno != EINTR)
    {
        DieNowWithMessage("wait response", OT_EXIT_FAILURE);
    }
#endif // OPENTHREAD_POSIX_VIRTUAL_TIME

exit:
    return error;
}

void HdlcInterface::UpdateFdSet(void *aMainloopContext)
{
    otSysMainloopContext *context = reinterpret_cast<otSysMainloopContext *>(aMainloopContext);

    assert(context != nullptr);

    FD_SET(mSockFd, &context->mReadFdSet);

    if (context->mMaxFd < mSockFd)
    {
        context->mMaxFd = mSockFd;
    }
}

void HdlcInterface::Process(const void *aMainloopContext)
{
#if OPENTHREAD_POSIX_VIRTUAL_TIME
    /**
     * Process read data (decode the data).
     *
     * Is intended only for virtual time simulation. Its behavior is similar to `Read()` but instead of
     * reading the data from the radio socket, it uses the given data in @p `event`.
     */
    const VirtualTimeEvent *event = reinterpret_cast<const VirtualTimeEvent *>(aMainloopContext);

    assert(event != nullptr);

    Decode(event->mData, event->mDataLength);
#else
    const otSysMainloopContext *context = reinterpret_cast<const otSysMainloopContext *>(aMainloopContext);

    assert(context != nullptr);

    if (FD_ISSET(mSockFd, &context->mReadFdSet))
    {
        Read();
    }
#endif
}

otError HdlcInterface::WaitForWritable(void)
{
    otError        error   = OT_ERROR_NONE;
    struct timeval timeout = {kMaxWaitTime / 1000, (kMaxWaitTime % 1000) * 1000};
    uint64_t       now     = otPlatTimeGet();
    uint64_t       end     = now + kMaxWaitTime * OT_US_PER_MS;
    fd_set         writeFds;
    fd_set         errorFds;
    int            rval;

    while (true)
    {
        FD_ZERO(&writeFds);
        FD_ZERO(&errorFds);
        FD_SET(mSockFd, &writeFds);
        FD_SET(mSockFd, &errorFds);

        rval = select(mSockFd + 1, nullptr, &writeFds, &errorFds, &timeout);

        if (rval > 0)
        {
            if (FD_ISSET(mSockFd, &writeFds))
            {
                ExitNow();
            }
            else if (FD_ISSET(mSockFd, &errorFds))
            {
                DieNow(OT_EXIT_FAILURE);
            }
            else
            {
                assert(false);
            }
        }
        else if ((rval < 0) && (errno != EINTR))
        {
            DieNow(OT_EXIT_ERROR_ERRNO);
        }

        now = otPlatTimeGet();

        if (end > now)
        {
            uint64_t remain = end - now;

            timeout.tv_sec  = static_cast<time_t>(remain / OT_US_PER_S);
            timeout.tv_usec = static_cast<suseconds_t>(remain % OT_US_PER_S);
        }
        else
        {
            break;
        }
    }

    error = OT_ERROR_FAILED;

exit:
    return error;
}

int HdlcInterface::OpenFile(const Url::Url &aRadioUrl)
{
    int fd   = -1;
    int rval = 0;

    LogCrit("@@@ 4.1");
    fd = open(aRadioUrl.GetPath(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    LogCrit("@@@ 4.2");
    if (fd == -1)
    {
        LogCrit("@@@ 4.3");
        perror("open uart failed");
        ExitNow();
    }

    if (isatty(fd))
    {
        LogCrit("@@@ 4.4");
        struct termios tios;
        const char    *value;
        speed_t        speed;
        uint8_t        stopBit  = 1;
        uint32_t       baudrate = 115200;

        VerifyOrExit((rval = tcgetattr(fd, &tios)) == 0);
        LogCrit("@@@ 4.5");

        cfmakeraw(&tios);

        tios.c_cflag = CS8 | HUPCL | CREAD | CLOCAL;

        if ((value = aRadioUrl.GetValue("uart-parity")) != nullptr)
        {
            LogCrit("@@@ 4.6");
            if (strncmp(value, "odd", 3) == 0)
            {
                LogCrit("@@@ 4.7");
                tios.c_cflag |= PARENB;
                tios.c_cflag |= PARODD;
            }
            else if (strncmp(value, "even", 4) == 0)
            {
                LogCrit("@@@ 4.8");
                tios.c_cflag |= PARENB;
            }
            else
            {
                LogCrit("@@@ 4.9");
                DieNow(OT_EXIT_INVALID_ARGUMENTS);
            }
        }

        LogCrit("@@@ 4.10");
        IgnoreError(aRadioUrl.ParseUint8("uart-stop", stopBit));

        switch (stopBit)
        {
        case 1:
            LogCrit("@@@ 4.11");
            tios.c_cflag &= static_cast<unsigned long>(~CSTOPB);
            break;
        case 2:
            LogCrit("@@@ 4.12");
            tios.c_cflag |= CSTOPB;
            break;
        default:
            LogCrit("@@@ 4.13");
            DieNow(OT_EXIT_INVALID_ARGUMENTS);
            break;
        }

        LogCrit("@@@ 4.14");
        IgnoreError(aRadioUrl.ParseUint32("uart-baudrate", baudrate));

        switch (baudrate)
        {
        case 9600:
            LogCrit("@@@ 4.15");
            speed = B9600;
            break;
        case 19200:
            LogCrit("@@@ 4.16");
            speed = B19200;
            break;
        case 38400:
            LogCrit("@@@ 4.17");
            speed = B38400;
            break;
        case 57600:
            LogCrit("@@@ 4.18");
            speed = B57600;
            break;
        case 115200:
            LogCrit("@@@ 4.19");
            speed = B115200;
            break;
#ifdef B230400
        case 230400:
            LogCrit("@@@ 4.20");
            speed = B230400;
            break;
#endif
#ifdef B460800
        case 460800:
            LogCrit("@@@ 4.21");
            speed = B460800;
            break;
#endif
#ifdef B500000
        case 500000:
            LogCrit("@@@ 4.22");
            speed = B500000;
            break;
#endif
#ifdef B576000
        case 576000:
            LogCrit("@@@ 4.23");
            speed = B576000;
            break;
#endif
#ifdef B921600
        case 921600:
            LogCrit("@@@ 4.24");
            speed = B921600;
            break;
#endif
#ifdef B1000000
        case 1000000:
            LogCrit("@@@ 4.25");
            speed = B1000000;
            break;
#endif
#ifdef B1152000
        case 1152000:
            LogCrit("@@@ 4.26");
            speed = B1152000;
            break;
#endif
#ifdef B1500000
        case 1500000:
            LogCrit("@@@ 4.27");
            speed = B1500000;
            break;
#endif
#ifdef B2000000
        case 2000000:
            LogCrit("@@@ 4.28");
            speed = B2000000;
            break;
#endif
#ifdef B2500000
        case 2500000:
            LogCrit("@@@ 4.29");
            speed = B2500000;
            break;
#endif
#ifdef B3000000
        case 3000000:
            LogCrit("@@@ 4.30");
            speed = B3000000;
            break;
#endif
#ifdef B3500000
        case 3500000:
            LogCrit("@@@ 4.31");
            speed = B3500000;
            break;
#endif
#ifdef B4000000
        case 4000000:
            LogCrit("@@@ 4.32");
            speed = B4000000;
            break;
#endif
        default:
            LogCrit("@@@ 4.33");
            DieNow(OT_EXIT_INVALID_ARGUMENTS);
            break;
        }

        LogCrit("@@@ 4.33");
        mBaudRate = baudrate;

        if (aRadioUrl.HasParam("uart-flow-control"))
        {
            LogCrit("@@@ 4.34");
            tios.c_cflag |= CRTSCTS;
        }
        else if (aRadioUrl.HasParam("uart-init-deassert"))
        {
            LogCrit("@@@ 4.35");
            // When flow control is disabled, deassert DTR and RTS on init
#ifndef __APPLE__
            int flags;
#endif

            tios.c_cflag &= ~(CRTSCTS);

#ifndef __APPLE__
            // Deassert DTR and RTS
            flags = TIOCM_DTR | TIOCM_RTS;
            VerifyOrExit(ioctl(fd, TIOCMBIC, &flags) != -1, perror("tiocmbic"));
            LogCrit("@@@ 4.36");
#endif
        }

        LogCrit("@@@ 4.37");
        VerifyOrExit((rval = cfsetspeed(&tios, static_cast<speed_t>(speed))) == 0, perror("cfsetspeed"));
        LogCrit("@@@ 4.38");
        rval = tcsetattr(fd, TCSANOW, &tios);

#ifdef __APPLE__
        if (rval)
        {
            struct termios orig_tios;
            VerifyOrExit((rval = tcgetattr(fd, &orig_tios)) == 0, perror("tcgetattr"));
            VerifyOrExit((rval = cfsetispeed(&tios, cfgetispeed(&orig_tios))) == 0, perror("cfsetispeed"));
            VerifyOrExit((rval = cfsetospeed(&tios, cfgetospeed(&orig_tios))) == 0, perror("cfsetospeed"));
            VerifyOrExit((rval = tcsetattr(fd, TCSANOW, &tios)) == 0, perror("tcsetattr"));
            VerifyOrExit((rval = ioctl(fd, IOSSIOSPEED, &speed)) == 0, perror("ioctl IOSSIOSPEED"));
        }
#else  // __APPLE__
        VerifyOrExit(rval == 0, perror("tcsetattr"));
#endif // __APPLE__
        LogCrit("@@@ 4.39");
        VerifyOrExit((rval = tcflush(fd, TCIOFLUSH)) == 0);
        LogCrit("@@@ 4.40");
    }

exit:
    if (rval != 0)
    {
        LogCrit("@@@ 4.41");
        DieNow(OT_EXIT_FAILURE);
    }

    LogCrit("@@@ 4.42");
    return fd;
}

void HdlcInterface::CloseFile(void)
{
    VerifyOrExit(mSockFd != -1);

    VerifyOrExit(0 == close(mSockFd), perror("close RCP"));
    VerifyOrExit(-1 != wait(nullptr) || errno == ECHILD, perror("wait RCP"));

    mSockFd = -1;

exit:
    return;
}

#if OPENTHREAD_POSIX_CONFIG_RCP_PTY_ENABLE
int HdlcInterface::ForkPty(const Url::Url &aRadioUrl)
{
    int fd   = -1;
    int pid  = -1;
    int rval = -1;

    {
        struct termios tios;

        memset(&tios, 0, sizeof(tios));
        cfmakeraw(&tios);
        tios.c_cflag = CS8 | HUPCL | CREAD | CLOCAL;

        VerifyOrDie((pid = forkpty(&fd, nullptr, &tios, nullptr)) != -1, OT_EXIT_ERROR_ERRNO);
    }

    if (0 == pid)
    {
        constexpr int kMaxArguments = 32;
        char         *argv[kMaxArguments + 1];
        size_t        index = 0;

        argv[index++] = const_cast<char *>(aRadioUrl.GetPath());

        for (const char *arg = nullptr;
             index < OT_ARRAY_LENGTH(argv) && (arg = aRadioUrl.GetValue("forkpty-arg", arg)) != nullptr;
             argv[index++] = const_cast<char *>(arg))
        {
        }

        if (index < OT_ARRAY_LENGTH(argv))
        {
            argv[index] = nullptr;
        }
        else
        {
            DieNowWithMessage("Too many arguments!", OT_EXIT_INVALID_ARGUMENTS);
        }

        VerifyOrDie((rval = execvp(argv[0], argv)) != -1, OT_EXIT_ERROR_ERRNO);
    }
    else
    {
        VerifyOrDie((rval = fcntl(fd, F_GETFL)) != -1, OT_EXIT_ERROR_ERRNO);
        VerifyOrDie((rval = fcntl(fd, F_SETFL, rval | O_NONBLOCK | O_CLOEXEC)) != -1, OT_EXIT_ERROR_ERRNO);
    }

    return fd;
}
#endif // OPENTHREAD_POSIX_CONFIG_RCP_PTY_ENABLE

void HdlcInterface::HandleHdlcFrame(void *aContext, otError aError)
{
    static_cast<HdlcInterface *>(aContext)->HandleHdlcFrame(aError);
}

void HdlcInterface::HandleHdlcFrame(otError aError)
{
    VerifyOrExit((mReceiveFrameCallback != nullptr) && (mReceiveFrameBuffer != nullptr));

    mInterfaceMetrics.mTransferredFrameCount++;

    if (aError == OT_ERROR_NONE)
    {
        mInterfaceMetrics.mRxFrameCount++;
        mInterfaceMetrics.mRxFrameByteCount += mReceiveFrameBuffer->GetLength();
        mInterfaceMetrics.mTransferredValidFrameCount++;
        mReceiveFrameCallback(mReceiveFrameContext);
    }
    else
    {
        mInterfaceMetrics.mTransferredGarbageFrameCount++;
        mReceiveFrameBuffer->DiscardFrame();
        LogWarn("Error decoding hdlc frame: %s", otThreadErrorToString(aError));
    }

exit:
    return;
}

otError HdlcInterface::ResetConnection(void)
{
    otError  error = OT_ERROR_NONE;
    uint64_t end;

    if (mRadioUrl.HasParam("uart-reset"))
    {
        usleep(static_cast<useconds_t>(kRemoveRcpDelay) * OT_US_PER_MS);
        CloseFile();

        end = otPlatTimeGet() + kResetTimeout * OT_US_PER_MS;
        do
        {
            mSockFd = OpenFile(mRadioUrl);
            if (mSockFd != -1)
            {
                ExitNow();
            }
            usleep(static_cast<useconds_t>(kOpenFileDelay) * OT_US_PER_MS);
        } while (end > otPlatTimeGet());

        LogCrit("Failed to reopen UART connection after resetting the RCP device.");
        error = OT_ERROR_FAILED;
    }

exit:
    return error;
}

} // namespace Posix
} // namespace ot
#endif // OPENTHREAD_POSIX_CONFIG_SPINEL_HDLC_INTERFACE_ENABLE
