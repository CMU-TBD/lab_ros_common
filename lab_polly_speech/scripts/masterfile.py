#!/usr/bin/env python

import sys
import rospkg
import os

# yield accurate current working directory
rospack = rospkg.RosPack()
directory = os.path.join(rospack.get_path("lab_polly_speech"), "scripts/masterfile.txt")

 # class for key tuples to .txt file masterfile
class Item:
    speaker = ""
    text = ""
 
    def __init__(self, speaker, text):
        self.speaker = speaker
        self.text = text
 
# .txt file hashtable
class HashTable:
     
    def __init__(self, size):
        self.tableSize = size

    # hash function     
    def hashing(self, item):
        return hash(item.speaker + item.text) % self.tableSize
 
    # check for duplicate hashes
    def isDuplicate(self, hash):
        with open(directory) as f:
            lines = f.readlines()
            for line in lines:
                
                # exact match, don't do anything
                if line == ("(%s, %s): %i.mp3\n" % (item.speaker, item.text, hash)):
                    return

                parameters = line.split(':')
                index = parameters[1].split('.')

                # we have a match in hash value
                if index[0].lstrip() == str(hash):
                    return True
            return False

    # insert key into .txt file
    def insert(self, item):
        hash = self.hashing(item)           
        x = 0

        with open(directory) as f:
            lines = f.readlines()
            x = f.tell()
            for line in lines:
                
                # edge case with empty file
                if line == '\n' and len(lines) == 1:
                    f = open(directory, 'rw+')
                    f.write("(%s, %s): %i.mp3\n" % (item.speaker, item.text, hash))
                    f.close()
                    return
                # remove extra whitespace
                elif line == '\n':
                    f = open(directory, 'rw+')
                    f.truncate()
                    return
                
                parameters = line.split(':')
                index = parameters[1].split('.')

                # no need to insert if already there
                if line == ("(%s, %s): %i.mp3\n" % (item.speaker, item.text, hash)):
                    return

                # use open addressing until unused hash
                # or duplicate is found
                elif index[0].lstrip() == str(hash):
                    while self.isDuplicate(hash):
                        hash += 1
                f.close()
        
        # write line if not a duplicate
        f = open(directory, 'rw+')
        f.seek(x)
        f.write("(%s, %s): %i.mp3\n" % (item.speaker, item.text, hash))
        f.truncate()
        f.close()

    # return mp3 value of item
    def find(self, item):
        hash = self.hashing(item) 
        with open(directory) as f:
            lines = f.readlines()

            # compare keys and return hash if valid
            for line in lines:
                if line.split(":")[0] == ("(%s, %s)" % (item.speaker, item.text)):
                    return line.split(":")[1].lstrip().rstrip()
            return None

    # delete entry from table
    def delete(self, item):
        hash = self.hashing(item)
        with open(directory) as f:
            lines = f.readlines()
            f = open(directory, 'rw+')
            
            # exclude given line and truncate file
            for line in lines:
                if line != ("(%s, %s): %i.mp3\n" % (item.speaker, item.text, hash)):
                    f.write(line)
            f.truncate()
            f.close()
        return

    # return number of entries in file
    def numEntries(self):
        with open(directory) as f:
            for i, j in enumerate(f):
                pass
        return i+1
 
