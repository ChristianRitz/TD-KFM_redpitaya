#!/usr/bin/env python

import json, time, os
import numpy as np
from registers import *
import cli
try:
    # for Python2
    import Tkinter as tk
    import Tkinter.filedialog as filedialog
except ImportError:
    # for Python3
    import tkinter as tk
    import tkinter.filedialog as filedialog
import threading
import paramiko

from gui_config import gui_elements,gui_setup


sig_figs = 6
def round_to_signif(val,n=sig_figs):
    """Round number to n significant figures."""
    if val == 0: return 0
    mag = 10**np.floor(np.log10(np.abs(val)))
    return np.around(10**n*val/mag)*mag/10**n



class MainWindow(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.parent = parent
        self.elements = gui_elements
        
        #top-bar with buttons
        self.button_frame = tk.Frame(self.parent, background='white')
        self.button_frame.grid(row=0, column=0, sticky='EW')

        #connect to device button
        self.connect_button_txt = tk.StringVar()
        self.connect_button_txt.set('connect')
        self.connect_button = tk.Button(self.button_frame, command=self.connect_to_device, textvariable=self.connect_button_txt)
        self.connect_button.configure(bg='darkred', fg='white', activebackground='gray33', activeforeground='white', relief='flat')
        self.connect_button.pack(side='right', padx=10, pady=5)

        element = tk.Label(self.button_frame, text=gui_setup['subtitle'], bg='white', fg='gray33')
        element.pack(side='left', padx=10, pady=10)

        self.window = tk.Frame(self.parent, bg='white')
        self.window.grid(row=1, column=0)

        #get default parameters
        with open(defaultdir+'/config/default.json', 'r') as infile:
            default_params = json.load(infile)
        default_params['Start']['run program'] = False
        default_params['Start']['reset estimations'] = False

        #create view
        for i,groupname in enumerate(self.elements.keys()):
            self.createPanel(self.window, groupname, i, default_params)
        
        #add line for buttons at the bottom
        self.n_panels = len(self.elements.keys())-1
        self.button_frame_bottom = tk.Frame(self.window, background='white')
        self.button_frame_bottom.grid(row=self.n_panels+2, column=0, sticky='EW')

        #read from device button
        element = tk.Button(self.button_frame_bottom, command=self.read_all, text='read all')
        element.configure(bg=gui_setup['maincolor'], fg='white', activebackground='gray33', activeforeground='white', relief='flat')
        element.pack(side='left', padx=10, pady=5)        
        
        #write to device button
        element = tk.Button(self.button_frame_bottom, command=self.write_all, text='write all')
        element.configure(bg=gui_setup['maincolor'], fg='white', activebackground='gray33', activeforeground='white', relief='flat')
        element.pack(side='left', padx=10, pady=5)

        #save parameters button
        element = tk.Button(self.button_frame_bottom, command=self.save_parameters, text='save')
        element.configure(bg=gui_setup['maincolor'], fg='white', activebackground='gray33', activeforeground='white', relief='flat')
        element.pack(side='right', padx=10, pady=5)

        #load parameters button
        element = tk.Button(self.button_frame_bottom, command=self.load_parameters, text='load')
        element.configure(bg=gui_setup['maincolor'], fg='white', activebackground='gray33', activeforeground='white', relief='flat')
        element.pack(side='right', padx=10, pady=5)

        #add placeholder at the bottom
        self.place_holder = tk.Label(self.window, text=gui_setup['placeholder'], width=60, bg='white', fg='gray55', anchor='w')
        self.place_holder.grid(row=self.n_panels+3, padx=10, pady=5)
        
        #set running variable to false
        self.running = False

        #set connection variable to false
        self.device_connected = False





    def save_parameters(self):
        save_dict = {}
        for groupname in self.elements.keys():
            save_dict[groupname] = {}
            for name in self.elements[groupname].keys():
                if name in ['run program','reset estimations']: continue
                save_dict[groupname][name] = self.elements[groupname][name]['value'].get()
        savepath = filedialog.asksaveasfilename(title='save parameters', 
                                               initialdir=defaultdir+'/config',defaultextension='.json',
                                               filetypes=[('json file', ".json"),('All Files', '*')])
        with open(savepath, 'w') as outfile:
            json.dump(save_dict, outfile)


    def load_parameters(self, filepath=None):
        if filepath==None:
            loadpath = filedialog.askopenfilename(title='load parameters', 
                                                  initialdir=defaultdir+'/config',
                                                  filetypes=[('json file', ".json"),('All Files', '*')])
        with open(loadpath, 'r') as infile:
            params = json.load(infile)

        for groupname in self.elements.keys():
            for name in self.elements[groupname].keys():
                if name in ['run program', 'reset estimations']: continue
                self.elements[groupname][name]['value'].set(params[groupname][name])
        


    def start_program(self):    
        command = "python3 /root/TD-KFM_redpitaya/interface/run_main.py"
        stdin, stdout, stderr = self.ssh.exec_command(command)     



    def connect_to_device(self):
        if self.device_connected:
            if self.check_connection():
                print('device is already connected')
                return
        
        with open(defaultdir+'/config/ssh_info.json') as f:
            self.ssh_info = json.load(f)
        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.load_system_host_keys()
        
        try:
            self.ssh.connect(
                self.ssh_info['HOST'], 
                port = self.ssh_info['PORT'],
                username = self.ssh_info['USER'], 
                password = self.ssh_info['PASS'],
                timeout = 1,
                )
            self.device_connected = True
            self.connect_button.configure(bg='yellow green')
            self.connect_button_txt.set('connected')
            print('connected to',self.ssh_info['USER']+'\u0040'+self.ssh_info['HOST'])

            try: self.read_all()
            except: 
                print('there was an error while reading out the parameters')
        except:
            print('could not connect to',self.ssh_info['USER']+'\u0040'+self.ssh_info['HOST']+' - check your config/ssh.json')


    def check_connection(self, fast=False):
        if fast: return self.device_connected

        try: self.ssh.exec_command('ls', timeout = 1)
        except:
            print('device not connected')
            self.ssh.close()
            self.device_connected=False
            self.connect_button.configure(bg='darkred')
            self.connect_button_txt.set('connect')
            return False
        return True


    def check_running(self, fast=False):
        if fast: return self.running
        
        command = 'pgrep kfm_controller'
        stdin, stdout, stderr = self.ssh.exec_command(command, timeout=1)
        if stdout.readlines() == []:
            print('controller not running')
            self.running = False
            self.elements['Start']['run program']['value'].set(False)
            return False
        else:
            self.running = True
            self.elements['Start']['run program']['value'].set(True)
            return True


    def read_all(self):
        if not self.check_connection(): return
        if not self.check_running(): return
        for groupname in self.elements.keys():
            for name in self.elements[groupname].keys():
                reg = self.elements[groupname][name]['reg']
                if self.elements[groupname][name]['type'] == 'Checkbox':
                    self.read_bool(groupname, name, reg)
                elif self.elements[groupname][name]['type'] == 'Entry':
                    self.read_value(groupname, name, reg)

    def write_all(self):
        if not self.check_connection(): return
        if not self.check_running(): return
        for groupname in self.elements.keys():
            for name in self.elements[groupname].keys():
                if name in ['run program']: continue
                reg = self.elements[groupname][name]['reg']
                if self.elements[groupname][name]['type'] == 'Checkbox':
                    self.write_bool(groupname, name, reg)
                elif self.elements[groupname][name]['type'] == 'Entry':
                    self.write_value(groupname, name, reg)
        
    def read_value(self,groupname,name,reg):
        if not self.check_connection(fast=True): return 0
        if not self.check_running(fast=True): return 0
        value = cli.rp_get_multireg_float(self.ssh, reg)
        value = round_to_signif(value)
        print('read', name, '=', value)
        self.elements[groupname][name]['value'].set(value)
        return value

    def read_bool(self,groupname,name,reg):
        if not self.check_connection(fast=True): return 0
        if not self.check_running(fast=True): return 0
        value = cli.rp_get_register_bit(self.ssh, REG_FLAGS,  reg)
        print('read', name, '=', value)
        self.elements[groupname][name]['value'].set(value)
        return value

    def write_value(self,groupname,name,reg):
        value = self.elements[groupname][name]['value'].get()
        if not self.check_connection(fast=True): return
        if not self.check_running(fast=True): return
        print('write', name, '=', value)
        cli.rp_set_multireg_float(self.ssh, reg, value)

    def write_bool(self,groupname,name,reg):
        value = self.elements[groupname][name]['value'].get()
        if not self.check_connection(fast=True): return
        if name != 'run program': 
            if not self.check_running(fast=True): return
            cli.rp_set_register_bit(self.ssh, REG_FLAGS, reg, status = value)
            print('write', name, '=', value)

            if name == 'measure sampling rate' and value == 1:
                time.sleep(.2) #give some time to measure samplingrate
                value = cli.rp_get_multireg_float(self.ssh, INDEX_SAMPLINGRATE)
                value = round_to_signif(value)
                print('read sampling rate =', value)
                self.elements['State Observer']['sampling rate']['value'].set(value)

            #directly release button in some cases
            if name in ['reset estimations','measure sampling rate'] :
                self.elements[groupname][name]['value'].set(False)

        #special case: run button
        elif value == 1:
            #start the main loop
            if self.check_running():
                print('controller is already running')
                self.read_all()
                return
            print('initializing loop')
            self.running = True
            self.startthread = threading.Thread(target = self.start_program, daemon=True)
            self.startthread.start()
            time.sleep(2) #wait for RP to respond
            self.write_all()
        else:
            #terminate the main loop
            print('terminating loop')
            cli.rp_set_register_bit(self.ssh, REG_FLAGS, reg, status = value)
            self.running = False

                

    def add_increment(self,groupname,name,reg,multiplier):
        value = self.elements[groupname][name]['value'].get()
        if value == 0:
            increment = 0.1
        else:
            log10_val = np.log10(np.abs(value))-1
            increment = 10**np.floor(log10_val)
            if log10_val%1==0 and np.sign(multiplier)*np.sign(value)==-1:
                increment /= 10
        value += increment * np.sign(multiplier)
        #round to n significant figures to avoid strange numbers       
        value = round_to_signif(value)
        self.elements[groupname][name]['value'].set(value)
        self.write_value(groupname,name,reg)


    def createPanel(self, parent, groupname, pos, default_params):
        panel = tk.LabelFrame(parent, fg=gui_setup['maincolor'], text=groupname, padx=10, pady=10, bg='white')
        panel.grid(row=pos, column = 0, sticky='EW', padx=10, pady=10)

        names = self.elements[groupname]
            
        for i,name in enumerate(names):
            reg = self.elements[groupname][name]['reg']
            tk.Label(panel, text = name, width='20', anchor='w', bg='white', fg='gray33').grid(row=i, column=0, sticky='W')
            if self.elements[groupname][name]['type'] == 'Checkbox':
                #element is a boolean so create a checkbutton
                self.elements[groupname][name]['value'] = tk.BooleanVar(value=default_params[groupname][name])
                entry=tk.Checkbutton(panel, bg='gray90', selectcolor=gui_setup['maincolor'], indicatoron=0, width=2, offrelief='flat', bd=0, pady=0, padx=0)
                entry.config(variable=self.elements[groupname][name]['value'])
                entry.config(command=lambda groupname=str(groupname), name=str(name), reg=reg: self.write_bool(groupname, name, reg))
                entry.grid(row=i, column=1, sticky='W', pady=2)
            elif self.elements[groupname][name]['type'] == 'Entry':
                #element has a value so create an entry field
                self.elements[groupname][name]['value'] = tk.DoubleVar(value=default_params[groupname][name])
                entry = tk.Entry(panel, width=10, bg='white', fg='black', text=self.elements[groupname][name]['value'])
                entry.grid(row=i, column=1, sticky='W', pady=2)
                entry.bind('<Return>', lambda _, groupname=str(groupname), name=str(name), reg=reg: self.write_value(groupname, name, reg))
                entry.bind('<KP_Enter>', lambda _, groupname=str(groupname), name=str(name), reg=reg: self.write_value(groupname, name, reg))
                entry.bind('<Up>', lambda _, groupname=str(groupname), name=str(name), reg=reg: self.add_increment(groupname, name, reg, 1))
                entry.bind('<Down>', lambda _, groupname=str(groupname), name=str(name), reg=reg: self.add_increment(groupname, name, reg, -1))
                tk.Label(panel, text = self.elements[groupname][name]['unit'], bg='white', fg='gray33').grid(row=i,column=2, sticky = 'W')
            else:
                #input method not known
                #use entry only as label and add no input
                continue



def close_app():
    root.quit()


if __name__ == "__main__":
    root = tk.Tk()
    

    defaultdir =  os.path.abspath(os.path.join(__file__ ,"../.."))

    root.title(gui_setup['title'])
    icon = tk.PhotoImage(file = defaultdir+'/interface/icon.png')
    root.iconphoto(False, icon)
    MW = MainWindow(root)
    root.protocol('WM_DELETE_WINDOW', close_app)
    root.mainloop()
                

