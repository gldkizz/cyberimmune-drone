#!/usr/bin/env python3
'''
MAVProxy buttons Module

'''
import wx
import platform
import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time

from MAVProxy.modules.lib import wxconsole
#from MAVProxy.modules.lib import textconsole
#
import threading

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import multiproc

#from MAVProxy.modules.lib import wxsettings
#from MAVProxy.modules.lib.mp_menu import *
#
from MAVProxy.modules.mavproxy_buttons import buttons_frame

class ButtonsModule(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(ButtonsModule, self).__init__(mpstate, "buttons", "", public=True)
        self.mpstate = mpstate
        self.mpstate.buttons = self
        self.needs_unloading = False


        self.last_unload_check_time = time.time()
        self.unload_check_interval = 0.1  # seconds

        self.buttons_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        

        if platform.system() == 'Windows':
            self.child = threading.Thread(target=self.child_task)
        else:
            self.child = multiproc.Process(target=self.child_task)

        self.child.start()

        self.last_unload_check_time = time.time()
        self.unload_check_interval = 0.1  # seconds

    def unload(self):
        self.mpstate.buttons.close()
 
    def idle_task(self):
        now = time.time()
        if now - self.last_unload_check_time > self.unload_check_interval:
            self.last_unload_check_time = now
            if not self.child.is_alive():
                self.needs_unloading = True

    def child_task(self):
        '''child process - this holds GUI elements'''
        mp_util.child_close_fds()
        self.app = wx.App(False)
        self.app.frame = buttons_frame.ButtonsFrame(
            parent=None, id=wx.ID_ANY)
        self.app.frame.mpstate = self.mpstate
        self.app.SetExitOnFrameDelete(True)
        self.app.frame.Show()

        # start a thread to monitor the "close window" semaphore:
        class CloseWindowSemaphoreWatcher(threading.Thread):
            def __init__(self, task, sem):
                threading.Thread.__init__(self)
                self.task = task
                self.sem = sem

            def run(self):
                self.sem.acquire(True)
                self.task.app.ExitMainLoop()

        self.app.MainLoop()

    def close(self):
        '''close the Buttons window'''
        self.time_to_quit = True
        if platform.system() == 'Windows':
            self.child.join()
        else:
            self.child.terminate()

 
def init(mpstate):
    '''initialise module'''
    return ButtonsModule(mpstate)
