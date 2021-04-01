#!/usr/bin/env python3
#
# Copyright (c) 2014, Altera Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Altera Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED.  IN NO EVENT SHALL ALTERA CORPORATION BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import sys
import re
import glob
import argparse
import textwrap
import subprocess
import time

MAX_PARTITIONS = 4

# Globals
loopback_dev_used = []
mounted_fs = []

#
#  ######  #    #  #    #   ####    ####
#  #       #    #  ##   #  #    #  #
#  #####   #    #  # #  #  #        ####
#  #       #    #  #  # #  #            #
#  #       #    #  #   ##  #    #  #    #
#  #        ####   #    #   ####    ####
#

#==============================================================================
# Convert to bytes
def convert_size_from_unit(unit_size):

    factor = 1

    m = re.match("^[0-9]+[KMG]?$", unit_size, re.I)
    if m == None:
        print ("error: "+unit_size+": malformed expression")
        sys.exit(-1)
    else:
        munit = re.search("[KMG]+$", m.group(0), re.I)
        msize = re.search("^[0-9]+", m.group(0), re.I)

        if munit :
            unit = munit.group(0).upper()

            if unit == 'K':
                factor = 1024
            elif unit == 'M':
                factor = 1024*1024
            elif unit == 'G':
                factor = 1024*1024*1024

    # convert_str_to_int() takes care of handling exceptions
    size = int(convert_str_to_int(msize.group(0))*factor)

    return int(size)

#==============================================================================
# converts a string to int, with exception handling
def convert_str_to_int(string):

    try:
        integer = int(string)

    except ValueError:
        print ("error: "+string+": not a valid number")
        sys.exit(-1)

    return integer

#==============================================================================
# Checks the requested file system format is supported
def validate_format(fs_format):

    match = re.search("^(ext[2-4]|xfs|fat32|vfat|fat|none|raw)$", fs_format, re.I)
    if match:
        return True
    else:
        return False

#==============================================================================
# The switch '-P' can be used multiple times, this function checks one
# instance
# It returns a dictionary with the right entries
def parse_single_part_args(part):

    part_entries = {}
    part_entries['files'] = []

    p = re.compile("[a-zA-Z0-9]+=")

    for el in part.split(","):
        if p.match(el):
            key, value = el.split("=")
            #  need to test for a situation like key=, that is
            #! without a value.
            if value == None:
                print ("error: "+key+": no value found.")
                sys.exit(-1)

            # check that a valid key was used
            if key == 'num':
                part_entries[key] = convert_str_to_int(value)
            elif key == 'size':
                size = convert_size_from_unit(value)
                part_entries[key] = int(size)
            elif key == 'format':
                if validate_format(value):
                    part_entries[key] = value
                else:
                    print ("error:", value, "unknown format")
                    sys.exit(-1)
            elif key == 'type':
                part_entries[key] = value
            else:
                print ("error:", key,": unknown option")
                sys.exit(-1)
        else:
            part_entries['files'].append(el)

    return part_entries

#==============================================================================
# Parse all the arguments provided with all the '-P' switches
def parse_all_parts_args(part_args):

    part_entries = {}

    num_args = len(part_args)
    if num_args > MAX_PARTITIONS:
        print ("error: up to "+str(MAX_PARTITIONS)+" allowed")
        sys.exit(-1)

    for part in part_args:
        part_entry = parse_single_part_args(part)
        if part_entry['num'] in part_entries.keys():
            print ("error:"+str(part_entry['num'])+": partition already used")
            sys.exit(-1)

        part_entries[part_entry['num']] = part_entry

    return part_entries

#==============================================================================
# in some cases, a partition type (fdisk) can be inferred from the file system
# format, e.g. ext[2-4], type=83
def derive_fdisk_type_from_format(pformat):

    ptype = ""

    if re.match('^ext[2-4]|xfs$', pformat):
        ptype = '83'
    elif re.match('^vfat|fat|fat32$', pformat):
        ptype = 'b'
    else:
        print ("error:", pformat,": unknown format")
        sys.exit(-1)

    return ptype

#==============================================================================
# The partition type provided by the user is not in the format that fdisk
# expects. This function translates to fdisk type defs
def derive_fdisk_type_from_ptype(ptype):

    ptype = ""

    if re.match('^raw|none$', ptype):
        fdisk_type = 'A2'
    elif ptype == 'swap':
        fdisk_type = '84'
    else:
        print ("error:", ptype,": unknown type")
        sys.exit(-1)

    return fdisk_type

#==============================================================================
# This function checks the partition definitions and calculates the
# partition offsets
def check_and_update_part_entries(part_entries, image_size):

    entry = {}
    offset = 2048   # in blocks of 512 bytes
    total_size = 0


    for part in part_entries.keys():

        entry = part_entries[part]

        # we need to check if num, size and format are set
        # if type is not set but format is set, we can derive the type
        # as long as the format is not 'raw' or 'none'
        if 'size' not in entry:
            print ("error:", part, ": size must be specified")
            sys.exit(-1)
        if entry['size'] == 0:
            print ("error:", part, ": size is 0")
            sys.exit(-1)
        total_size = total_size + entry['size']

        if 'format' not in entry:
            if 'type' not in entry:
                print ("error:", part,": specify at least format or type")
                sys.exit(-1)

            part_entries[part]['fdisk_type'] = derive_fdisk_type_from_ptype(entry['type'])

        else: # format in  entry
            if 'type' not in entry:
                part_entries[part]['fdisk_type'] = derive_fdisk_type_from_format(entry['format'])
            else:
                part_entries[part]['fdisk_type'] = entry['type']

        # update offset
        part_entries[part]['start'] = offset # in sectors
        bsize = ( entry['size'] / 512 + ((entry['size'] % 512) != 0)*1)  # because size is in bytes
        offset = offset + bsize + 1

        # it is handy to save the size in blocks, as this is what fdisk needs
        part_entries[part]['bsize'] = bsize

    if total_size > image_size:
        print ("error: partitions are too big to fit in image")
        sys.exit(-1)

    return part_entries

#==============================================================================
# this script can only be run by the zuper user
def is_user_root():

    return (os.getuid() == 0)

#==============================================================================
# check if a file exists
def check_file_exists(filename):

    return os.path.isfile(filename)

#==============================================================================
# this function creates an empty image
def create_empty_image(image_name, image_size, force_erase_image):

    # first check if the image exists...
    if check_file_exists(image_name):
        if force_erase_image == False:
            yes_or_no = raw_input("the image "+image_name+" exists. Remove? [y|n] ")
        else:
            yes_or_no = 'Y'

        if yes_or_no == 'Y' or yes_or_no == 'y':
            try:
                os.remove(image_name)
            except OSError:
                print ("error: failed to remove "+image_name+". Exit")
                sys.exit(-1)
            print ("image removed")

        else:
            print ("user declined")
            return False

    # now we can proceed with the image creation
    # we'll create an empty image to speed things up...
    try:
        subprocess.check_output(["dd", "if=/dev/zero", "of="+image_name,"bs=1",
                                 "count=0", "seek="+str(image_size)],
                                stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError:
        print ("error: failed to create the image")
        sys.exit(-1)

    return True

#==============================================================================
# this function creates a loopback device
# it is assumed the file exists
# offset in bytes
def create_loopback(image_name, size, offset=0):

    try:
        if offset != 0:
            device = subprocess.check_output(
                   ["losetup", "--show", "-f", "-o "+str(offset),
                    "--sizelimit", str(size), image_name])
        else:
            device = subprocess.check_output(
                   ["losetup", "--show", "-f",
                    "--sizelimit", str(size), image_name])

    except subprocess.CalledProcessError:
        print ("error: failed to get a loopback device")
        clean_up()
        sys.exit(-1)

    # strip trailing \n
    device = str.rstrip(str(device.decode('utf-8')))
    # keep track of the devices used
    loopback_dev_used.append(device)

    return device

#==============================================================================
# this function deletes a loopback device
def delete_loopback(device):

    try:
        subprocess.check_output(["losetup", "-d", str(device)], stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError:
        return False

    # remove the device from the list
    loopback_dev_used.remove(device)

    return True

#==============================================================================
# clean up
def clean_up():

    for mp in mounted_fs:
        umount_fs(mp)

    for device in loopback_dev_used:
        if not delete_loopback(device):
            print ("error: could not delete loopback device", device)


    return 0

#==============================================================================
# this function creates the partition table
def create_partition_table(loopback, partition_entries):

    # our command list for fdisk
    cmd = ""
    # the number of questions asked bby fdisk, for one partition depebds
    #!on the number of partitions defined
    first_part = True

    # open up a pipe to fdisk
    try:
        p = subprocess.Popen(['fdisk',loopback,"-u"],
                             stdin=subprocess.PIPE,
                             stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE)
    except OSError:
        print ("error: fdisk: system error")
        clean_up()
        sys.exit(-1)
    except ValueError:
        print ("error: Popen: invalid args")
        clean_up()
        sys.exit(-1)

    for part in partition_entries.keys():
        pentry = partition_entries[part]
        # first we create the partition
        cmd = """\
n
p
"""+str(int(pentry['num']))+"""
"""+str(int(pentry['start']))+"""
+"""+str(int(pentry['bsize']))+"""
"""
        cmd = cmd.encode(encoding='UTF-8')
        p.stdin.write(cmd)

        # second we set the type
        if first_part:
            cmd = """\
t
"""+pentry['fdisk_type']+"""
"""
            first_part = False
        else:
            cmd = """\
t
"""+str(int(pentry['num']))+"""
"""+pentry['fdisk_type']+"""
"""
        cmd = cmd.encode(encoding='UTF-8')
        p.stdin.write(cmd)


    # we need to write and quit
    cmd = """
w
q
"""
#    p.stdin.write(cmd)
#    p.stdin.flush()
#    p.wait()
    cmd = cmd.encode(encoding='UTF-8')
    p.communicate(cmd)

    # sometimes the kernel does not reload the pattition table
    # a little help is needed
    if p.returncode != 0:
        pp = subprocess.run(["partprobe", loopback])
        print ("Partition return code [", pp.returncode, "]")
        if pp.returncode != 0:
            print ("error: could not reload the partition table from image")
            sys.exit(-1)
    return

#==============================================================================
# map format to a command
def get_mkfs_from_format(pformat):

    cmd = ""

    if re.search("^ext[2-4]$", pformat):
        cmd = "mkfs."+pformat
    elif re.search("fat|vfat|fat32", pformat):
        cmd = "mkfs.vfat"
    elif re.search("^xfs$", pformat):
        cmd = "mkfs.xfs"

    return cmd

#==============================================================================
# map format to a command parameter
def get_mkfs_params_from_format(pformat):

    params = ""

    if re.search("fat32", pformat):
        params = ["-F32","-I"]
    elif re.search("vfat", pformat):
        params = ["-I"]

    return params

#==============================================================================
# formats a vlock device
def format_partition(loopback, fs_format):

    cmd = get_mkfs_from_format(fs_format)
    params = get_mkfs_params_from_format(fs_format)
    if cmd:
        if params:
            p = subprocess.Popen([cmd, loopback, *params],
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 universal_newlines=True)
        else:
            p = subprocess.Popen([cmd, loopback],
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 universal_newlines=True)
        #RODO: add timeout?
        out,err = p.communicate()
        if p.returncode != 0:
            print ("error: format: failed. Return code=%d\n" % p.returncode)
            print ("params=%s, cmd=%s\n" % (params,cmd))
            print ("stdout=%s\nstderr=%s\n" % (out, err))
            clean_up()
            sys.exit(-1)

    return

def get_mountfs_from_format(pformat):
    format = pformat

    if re.search("fat32|fat", pformat):
        format = "vfat"

    return format
#==============================================================================
# mount a file system
#! returns the mnt point
def mount_fs(loopback, fs_format):

    mp = "/tmp/"+str(int(time.time()))+"_"+str(os.getpid())
    try:
        os.mkdir(mp)
    except OSError:
        print ("error: failed to create mount point (", mp,")")
        clean_up()
        sys.exit(-1)

    format = get_mountfs_from_format(fs_format)

    p = subprocess.Popen(["mount", "-t", format, loopback, mp],
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         universal_newlines=True)
    out, err = p.communicate()
    if p.returncode != 0:
        print ("error: mount: failed (", loopback, mp,")")
        print ("stdout=%s\nstderr=%s\n" % (out, err))
        clean_up()
        sys.exit(-1)

    # keep track of the mount points
    mounted_fs.append(mp)

    return mp

#==============================================================================
# unmount fs
def umount_fs(mp):

    time.sleep(3)
    p = subprocess.Popen(["umount", mp],
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         universal_newlines=True)
    out, err = p.communicate()
    if p.returncode != 0:
        print ("error: failed to umount", mp)
        print ("stdout=%s\nstderr=%s\n" % (out, err))
        sys.exit(-1)

    # update the list
    mounted_fs.remove(mp)

    return

#==============================================================================
#do a raw copy of files to a partition
def do_raw_copy(loopback, partition_data):

    offset = 0  # offset in bytes

    # below, stuff is just a file...
    for stuff in partition_data['files']:
        # we do accept FILES only, no directories please
        if os.path.isdir(stuff):
            print ("error:", stuff, ": can't copy dirs to raw partitions")
            clean_up()
            sys.exit(-1)

        # now dd the file:
        #! dd if=file of=loopback bs=1 seek=offset
        p = subprocess.Popen(["dd", "if="+stuff, "of="+loopback, "bs=1",
                             "seek="+str(offset)],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             universal_newlines=True)
        out, err = p.communicate()
        if p.returncode != 0:
            print ("error:", stuff, ": failed to do raw copy")
            print ("stdout=%s\nstderr=%s\n" % (out, err))
            clean_up()
            sys.exit(-1)

        # handle offset
        offset = offset + os.stat(stuff).st_size

    return

#==============================================================================
# copy files over a file system
def do_copy(loopback, partition_data):

    mp = mount_fs(loopback, partition_data['format'])
    for stuff in partition_data['files']:
        if os.path.isdir(stuff):
            stuff = stuff+"/*"

        # some file systems have limited flags like FAT
        if re.search("^fat|vfat|fat32$", partition_data['format']):
            cp_opt = "-rt"
        else:
            cp_opt = "-at"

        # as we need to do UNIX path expansion, we'll use the class glob,
        #! so we need to call cp with the option -t, such that the destination
        #! directory can be specified first. The list returned by glob can then
        #! be added to the list of args passed to Popen
        try:
            p = subprocess.Popen(["cp", cp_opt, mp ] + glob.glob(stuff),
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 universal_newlines=True)
            out, err = p.communicate()
            if p.returncode:
                raise Exception([])
        except Exception:
            print ("error: failed to copy", stuff)
            print ("stdout=%s\nstderr=%s\n" % (out, err))
            clean_up()
            sys.exit(-1)

    umount_fs(mp)

    return

#==============================================================================
# copy files to  a partition
#! takes care of the format, if raw|none use dd
def copy_files_to_partition(loopback, partition_data):

    if re.search("raw|none", partition_data['format']):
        # RAW patition, nothin to mount, the files must be
        #! dd'ed in. ONLY files allowed, no directory
        # if multiple files are provided, they are dd'ed one after another,
        #! no GAP. If not acceptable, one file should be passed, as an image
        do_raw_copy(loopback, partition_data)
    else:
        do_copy(loopback, partition_data)

    return

#==============================================================================
# create, formats and copt files to partition
def do_partition(partition, image_name):

    print("do_partition entry")
    offset_bytes = int(partition['start'] * 512)

    if partition['format'] == "fat32" and partition['size'] < 33554432:
        print ("error: Unable to create a fat32 partition size < 32MB")
        sys.exit(-1)

    loopback = create_loopback(image_name, partition['size'], offset_bytes)
    format_partition(loopback, partition['format'])
    copy_files_to_partition(loopback, partition)
    time.sleep(3)
    if not delete_loopback(loopback):
        clean_up()
        sys.exit(-1)

    return

#==============================================================================
def create_image(image_name, image_size, partition_entries, force_erase_image):

    print ("info: creating the image "+image_name)
    # first we need an empty image
    if not create_empty_image(image_name, image_size, force_erase_image):
        print ("error: the image file could not be created")
        sys.exit(-1)

    # second, we'll create the partition table
    print ("info: creating the partition table")
    print ("image_name set to ", image_name)
    loopback = create_loopback(image_name, image_size)
    create_partition_table(loopback, partition_entries)
    print ("now delete loopback")
    delete_loopback(loopback)

    # now we iterate over the partitions
    print ("info: processing partitions...")
    for part in partition_entries.keys():
        print ("     partition #"+str(part)+"...")
        do_partition(partition_entries[part], image_name)

    return

#==============================================================================
#==============================================================================
#
#   ####    #####    ##    #####    #####
#  #          #     #  #   #    #     #
#   ####      #    #    #  #    #     #
#       #     #    ######  #####      #
#  #    #     #    #    #  #   #      #
#   ####      #    #    #  #    #     #
#
part_entries = []

# arguments
parser = argparse.ArgumentParser(description='Creates an SD card image for Altera\'s SoCFPGA SoC\'s',
                                 epilog = textwrap.dedent('''\
Usage: PROG [-h] -P <partition info> [-P ...]
-P
'''
))
parser.add_argument('-P', dest='part_args', action='append',
                    help='''specifies a partition. May be used multiple times.
                            file[,file,...],num=<part_num>,format=<vfat|fat32|ext[2-4]|xfs|raw>,
                            size=<num[K|M|G]>[,type=ID]''')
parser.add_argument('-s', dest='size', action='store',
                    default='8G', help='specifies the size of the image. Units K|M|G can be used.')
parser.add_argument('-n', dest='image_name', action='store',
                    default='somename.img', help='specifies the name of the image.')
parser.add_argument('-f', dest='force_erase_image', action='store_true',
                    default=False, help='deletes the image file if exists')
args = parser.parse_args()

# Only root can do this
if not is_user_root():
    print ("error: only root can do this...")
    sys.exit(-1)

# A few checks
part_entries = parse_all_parts_args(args.part_args)
image_size = int(convert_size_from_unit(args.size))
part_entries = check_and_update_part_entries(part_entries, image_size)

# we now have what we need
create_image(args.image_name, image_size, part_entries, args.force_erase_image)
print ("info: image created, file name is ", args.image_name)

