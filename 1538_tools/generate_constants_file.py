#!/usr/bin/env python3

import argparse
import subprocess

MAC_GREP_CONSTANT_COMMAND = "grep -Ihor \"CONSTANT([^)]*)\" {}"

MAC_GREP_COMMENTED_CONSTANT = "grep -Ihor \"//.*CONSTANT([^)]*)\" {}"

TMP_CONSTANTS_FILE = ".tmp_const"
TMP_COMMENTED_FILE = ".tmp_comment"

def constants_defined_mac(src_dir):
    """
    """
    grep_cmd = MAC_GREP_CONSTANT_COMMAND.format(src_dir) + " > " + TMP_CONSTANTS_FILE
    rm_cmd = "rm " + TMP_CONSTANTS_FILE

    subprocess.run(grep_cmd,shell=True)

    constants = {}
    lines = []
    with open(TMP_CONSTANTS_FILE,'r') as reader:
        lines = reader.readlines()
    
    for line in lines:
        constant = line.split('"')[1]
        if constant in constants:
            ct = constants[constant]
            constants[constant] = ct + 1
        else:
            constants[constant] = 1
            
    subprocess.run(rm_cmd,shell=True)

    return constants

def constants_commented_mac(src_dir,constants):
    """
    """
    grep_comments = MAC_GREP_COMMENTED_CONSTANT.format(src_dir) + " > " + TMP_COMMENTED_FILE
    grep_constants = MAC_GREP_CONSTANT_COMMAND.format(TMP_COMMENTED_FILE) + " > " + TMP_CONSTANTS_FILE
    rm_comments = "rm " + TMP_COMMENTED_FILE
    rm_constants = "rm " + TMP_CONSTANTS_FILE

    subprocess.run(grep_comments,shell=True)

    subprocess.run(grep_constants,shell=True)
    subprocess.run(rm_comments,shell=True)

    lines = []
    with open(TMP_CONSTANTS_FILE,'r') as reader:
        lines = reader.readlines()
    
    for line in lines:
        constant = line.split('"')[1]
        if constant in constants:
            ct = constants[constant]
            if ct == 1:
                del constants[constant]
            else:
                constants[constant] = ct - 1
        else:
            print("warning when identifying constant {}".format(constant))
    
    subprocess.run(rm_constants,shell=True)

    return constants

def compare_to_constants_file(constants, const_file):
    """
    
    """
    lines = []
    with open(const_file,'r') as reader:
        lines = reader.readlines()

    file_constants = []
    for line in lines:
        if line.startswith("#"):
            continue
        if "=" in line:
            file_constants.append(line.split('=')[0].strip())
    
    for constant in constants:
        if constant not in file_constants:
            print('WARNING: constant {} not present'.format(constant))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="verify constants required against constants file")
    parser.add_argument('--src', '-s', dest="src_dir", required=True,help="path to robot src directory")
    parser.add_argument('--constants','-c', dest="constants_file", required=True,help="constants file to check against")

    args = parser.parse_args()

    constants = constants_defined_mac(args.src_dir)
    constants = constants_commented_mac(args.src_dir,constants)

    compare_to_constants_file(constants,args.constants_file)