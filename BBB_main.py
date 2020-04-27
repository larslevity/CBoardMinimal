
""" Main function running on BBB """
from __future__ import print_function

import sys
import time
import logging
import errno

from socket import error as SocketError

from Src.Management import timeout
from Src.Management import exception
from Src.Management.thread_communication import sys_config


from Src.Communication import client
from Src.Communication import printer

from Src.Controller import lowlevel_controller


logPath = "log/"
fileName = 'testlog'
logFormatter = logging.Formatter(
    "%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s")
rootLogger = logging.getLogger()
rootLogger.setLevel(logging.INFO)

fileHandler = logging.FileHandler("{0}/{1}.log".format(logPath, fileName))
fileHandler.setFormatter(logFormatter)
rootLogger.addHandler(fileHandler)
consoleHandler = logging.StreamHandler()
consoleHandler.setFormatter(logFormatter)
rootLogger.addHandler(consoleHandler)


# ------------ CAMERA INIT

def init_monitor_connection():
    plotsock = None
    pc_ip = '134.28.136.131'  #'192.168.7.1'

    with timeout.timeout(2):
        try:
            plotsock = client.LivePlotterSocket(pc_ip)
            rootLogger.info("Connected to LivePlotter Server")
        except exception.TimeoutError:
            rootLogger.info("LivePlotter Server not found")
        except SocketError as err:
            if err.errno == errno.ECONNREFUSED:
                rootLogger.info("LivePlotter Server refused connection")
            elif err.errno == errno.EADDRINUSE:
                rootLogger.info("LivePlotter Server already in Use")
            else:
                raise

    return plotsock


def main():
    try:
        rootLogger.info('Starting LowLevelController ...')
        lowlevelctr = lowlevel_controller.LowLevelController()
        lowlevelctr.start()
        time.sleep(.5)  # wait to init
        sys_config.IMUsConnected = lowlevelctr.is_imu_in_use()

        rootLogger.info('Searching for external devices in periphere...')
        plotsock = init_monitor_connection()
        sys_config.LivePlotter = plotsock

        rootLogger.info(sys_config)

        if sys_config.LivePlotter:
            rootLogger.info('Starting GUI Printer ...')
            guiPrinter = printer.GUIPrinter(sys_config.LivePlotter)
            guiPrinter.setDaemon(True)
            guiPrinter.start()

# %%

        # ...

# %%

        lowlevelctr.join()
        if sys_config.LivePlotter:
            guiPrinter.join()

    except KeyboardInterrupt:
        rootLogger.info('KeyboardInterrupt ...')

    finally:
        lowlevelctr.kill()
        if sys_config.LivePlotter:
            guiPrinter.kill()

    rootLogger.info('Program done.')
    sys.exit(0)


if __name__ == '__main__':
    main()
