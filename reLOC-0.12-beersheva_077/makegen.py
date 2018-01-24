# Makegen - Makefile Generator
# Version: 1.2
# (C) Copyright 2010-2012 Pavel Surynek
# http://www.surynek.com
# pavel.surynek@mff.cuni.cz

import fileinput
import string
import shutil
import os
import sys

class ModuleRecord:
  pass

def print_intro():
  print "Makegen 1.2 - Makefile Generator"
  print "(C) Copyright 2010-2012 Pavel Surynek"
  print "---------------------------------------"

include_dirs = set()
program_dirs = set()
modules = list()
selected_modules = list()

def get_first_word(line):
  begin = -1
  end = -1
  for i in range(len(line)):
    if begin == -1:
      if line[i] not in string.whitespace:
        begin = i
    else:
      if end == -1:
        if line[i] in string.whitespace:
          end = i
          break
  if begin == -1:
    return ""
  else:
    if end == -1:
      end = len(line)
  return line[begin:end]


def get_remaining_words(line):
  begin = -1
  end = -1
  for i in range(len(line)):
    if begin == -1:
      if line[i] not in string.whitespace:
        begin = i
    else:
      if end == -1:
        if line[i] in string.whitespace:
          end = i
      else:
        if line[i] not in string.whitespace:
          end = i
          break
  if begin == -1 or end == -1:
    return ""
  return line[end:len(line)]

              
def load_modules(modules_file):
  while True:
    module_record = ModuleRecord()
    module_identifier = modules_file.readline()
    
    while module_identifier != "" and get_first_word(module_identifier) == "":
      module_identifier = modules_file.readline()
      
    if module_identifier == "":
      break

    module_record.identifier = get_first_word(module_identifier)

    module_type = modules_file.readline()

    if string.find(module_type, "program") == 0:
      module_record.type = "program"
      module_directory = modules_file.readline()
      module_record.directory = get_first_word(module_directory)
      program_dirs.add(module_record.directory)

      module_headers = modules_file.readline()
      module_record.headers = list()

      header = get_first_word(module_headers)
      while header != "":
        module_headers = get_remaining_words(module_headers)
        module_record.headers.append(header)
        include_dirs.add(module_record.directory)
        header = get_first_word(module_headers)
      
      module_sources = modules_file.readline()
      module_record.sources = list()

      source = get_first_word(module_sources)
      while source != "":
        module_sources = get_remaining_words(module_sources)
        module_record.sources.append(source)
        source = get_first_word(module_sources)      

    elif string.find(module_type, "application") == 0:
      module_record.type = "application"
      module_directory = modules_file.readline()
      module_record.directory = get_first_word(module_directory)
      program_dirs.add(module_record.directory)

      module_headers = modules_file.readline()
      module_record.headers = list()

      header = get_first_word(module_headers)
      while header != "":
        module_headers = get_remaining_words(module_headers)
        module_record.headers.append(header)
        include_dirs.add(module_record.directory)
        header = get_first_word(module_headers)
      
      module_sources = modules_file.readline()
      module_record.sources = list()

      source = get_first_word(module_sources)
      while source != "":
        module_sources = get_remaining_words(module_sources)
        module_record.sources.append(source)
        source = get_first_word(module_sources)

      module_libraries = modules_file.readline()
      module_record.libraries = list()

      library = get_first_word(module_libraries)
      while library != "":
        module_libraries = get_remaining_words(module_libraries)
        module_record.libraries.append(library)
        library = get_first_word(module_libraries)
      
    elif string.find(module_type, "files") == 0:
      module_record.type = "files"
      module_directory = modules_file.readline()
      module_record.directory = get_first_word(module_directory)

      module_files = modules_file.readline()
      module_record.files = list()

      file = get_first_word(module_files)
      while file != "":
        module_files = get_remaining_words(module_files)
        module_record.files.append(file)
        file = get_first_word(module_files)
        
    else:
      print "Error: Unrecognized module type: " + module_type
      break;

    modules.append(module_record)


def load_selection(selection_file):
  for line in selection_file:
    module = get_first_word(line)
    selected_modules.append(module)

  for md in modules:
    sel = False
    for smd in selected_modules:
      if smd == md.identifier:
        sel = True
        break
    md.selected = sel


def construct_object_name_debug(module_directory, source):
  depth = 0
  start = 0

  pos = string.find(module_directory, "/", start)
  while pos >= 0:
    depth = depth + 1
    start = pos + 1
    pos = string.find(module_directory, "/", start)

  if source[0] == '/':
    prefix = ""
    for d in range(depth):
      prefix = prefix + "../"
    ext_pos = string.find(source, ".")
    return prefix + source[1:ext_pos] + ".o_dbg"
  else:
    ext_pos = string.find(source, ".")
    return source[0:ext_pos] + ".o_dbg"


def construct_object_name_optimized(module_directory, source):
  depth = 0
  start = 0

  pos = string.find(module_directory, "/", start)
  while pos >= 0:
    depth = depth + 1
    start = pos + 1
    pos = string.find(module_directory, "/", start)

  if source[0] == '/':
    prefix = ""
    for d in range(depth):
      prefix = prefix + "../"
    ext_pos = string.find(source, ".")
    return prefix + source[1:ext_pos] + ".o_opt"
  else:
    ext_pos = string.find(source, ".")
    return source[0:ext_pos] + ".o_opt"


def construct_header_name(module_directory, header):
  depth = 0
  start = 0

  pos = string.find(module_directory, "/", start)
  while pos >= 0:
    depth = depth + 1
    start = pos + 1
    pos = string.find(module_directory, "/", start)

  if header[0] == '/':
    prefix = ""
    for d in range(depth):
      prefix = prefix + "../"
    return prefix + header[1:len(header)]
  else:
    return header[0:len(header)]


def construct_source_name(module_directory, source):
  depth = 0
  start = 0

  pos = string.find(module_directory, "/", start)
  while pos >= 0:
    depth = depth + 1
    start = pos + 1
    pos = string.find(module_directory, "/", start)

  if source[0] == '/':
    prefix = ""
    for d in range(depth):
      prefix = prefix + "../"
    return prefix + source[1:len(source)]
  else:
    return source[0:len(source)]


def generate_rules(clean_pattern):
 for pd in program_dirs:
   create = False
   for md in modules:
     if md.selected and (md.type == "program" or md.type == "application"):
       if pd == md.directory:
         create = True

   if create:
     if pd[0] == '/':
       pd2 = pd[1:len(pd)]
     if os.path.isdir(pd2):
       print "Processing existent directory ... " + pd2
     else:
       print "Processing non-existent directory ... " + pd2
       os.makedirs(pd2)
     makefile = open(pd2 + "/Makefile", "w")

     makefile.write("all: debug\n")
     makefile.write("\n")
     makefile.write("debug:\t")

     objects = set()

     for md in modules:
       if md.selected and md.type == "program":
         if pd == md.directory:
           for sr in md.sources:
             obj = construct_object_name_debug(md.directory, sr)
             objects.add(obj)
       if md.selected and md.type == "application":
         if pd == md.directory:
           for sr in md.sources:
             obj = construct_object_name_debug(md.directory, sr)
             objects.add(obj)

     for obj in objects:
       makefile.write(obj + " ")
     makefile.write("\n")

     for md in modules:
       if md.selected and md.type == "program":
         if pd == md.directory:
           makefile.write("\tg++ -o" + md.identifier)
           for sr in md.sources:
             obj = construct_object_name_debug(md.directory, sr)
             makefile.write(" " + obj)
           makefile.write("\n")
       if md.selected and md.type == "application":
         if pd == md.directory:
           makefile.write("\tg++ ")
           for lb in md.libraries:
             library = "-l" + lb + " "
             makefile.write(library)
           makefile.write("-o" + md.identifier)
           for sr in md.sources:
             obj = construct_object_name_debug(md.directory, sr)
             makefile.write(" " + obj)
           makefile.write("\n")

     makefile.write("\n")

     makefile.write("optimized:\t")

     objects = set()

     for md in modules:
       if md.selected and md.type == "program":
         if pd == md.directory:
           for sr in md.sources:
             obj = construct_object_name_optimized(md.directory, sr)
             objects.add(obj)
       if md.selected and md.type == "application":
         if pd == md.directory:
           for sr in md.sources:
             obj = construct_object_name_optimized(md.directory, sr)
             objects.add(obj)

     for obj in objects:
       makefile.write(obj + " ")
     makefile.write("\n")

     for md in modules:
       if md.selected and md.type == "program":
         if pd == md.directory:
           makefile.write("\tg++ -o" + md.identifier)
           for sr in md.sources:
             obj = construct_object_name_optimized(md.directory, sr)
             makefile.write(" " + obj)
           makefile.write("\n")
       if md.selected and md.type == "application":
         if pd == md.directory:
           makefile.write("\tg++ ")
           for lb in md.libraries:
             library = "-l" + lb + " "
             makefile.write(library)
           makefile.write("-o" + md.identifier)
           for sr in md.sources:
             obj = construct_object_name_optimized(md.directory, sr)
             makefile.write(" " + obj)
           makefile.write("\n")

     makefile.write("\n")

     finished = set();
     for md in modules:
       if md.selected and md.type == "program":
         if pd == md.directory:
           for sr in md.sources:
             obj = construct_object_name_debug(md.directory, sr)
             if obj not in finished:
               finished.add(obj)
               makefile.write(obj + ":\t")
               for hd in md.headers:
                 hdr = construct_header_name(md.directory, hd)
                 makefile.write(hdr + " ")
               src = construct_source_name(md.directory, sr)
               makefile.write(src + "\n")
               makefile.write("\tg++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -o" + obj + " " + src + "\n")
               makefile.write("\n")
       if md.selected and md.type == "application":
         if pd == md.directory:
           for sr in md.sources:
             obj = construct_object_name_debug(md.directory, sr)
             if obj not in finished:
               finished.add(obj)
               makefile.write(obj + ":\t")
               for hd in md.headers:
                 hdr = construct_header_name(md.directory, hd)
                 makefile.write(hdr + " ")
               src = construct_source_name(md.directory, sr)
               makefile.write(src + "\n")
               makefile.write("\tg++ ")
               makefile.write("-Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -g -c -o" + obj + " " + src + "\n")
               makefile.write("\n")

     makefile.write("\n")
     
     finished = set();
     for md in modules:
       if md.selected and md.type == "program":
         if pd == md.directory:
           for sr in md.sources:
             obj = construct_object_name_optimized(md.directory, sr)
             if obj not in finished:
               finished.add(obj)
               makefile.write(obj + ":\t")
               for hd in md.headers:
                 hdr = construct_header_name(md.directory, hd)
                 makefile.write(hdr + " ")
               src = construct_source_name(md.directory, sr)
               makefile.write(src + "\n")
               makefile.write("\tg++ -Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -o" + obj + " " + src + "\n")
               makefile.write("\n")
       if md.selected and md.type == "application":
         if pd == md.directory:
           for sr in md.sources:
             obj = construct_object_name_optimized(md.directory, sr)
             if obj not in finished:
               finished.add(obj)
               makefile.write(obj + ":\t")
               for hd in md.headers:
                 hdr = construct_header_name(md.directory, hd)
                 makefile.write(hdr + " ")
               src = construct_source_name(md.directory, sr)
               makefile.write(src + "\n")
               makefile.write("\tg++ ")
               makefile.write("-Wall -Wextra -pedantic -Wno-long-long -Wno-sign-compare -c -O3 -mtune=native -o" + obj + " " + src + "\n")
               makefile.write("\n")

     makefile.write("\n")
     makefile.write("clean:\n")
     makefile.write("\trm -f " + clean_pattern)
     for md in modules:
       if md.selected and (md.type == "program" or md.type == "application"):
         if pd == md.directory:
           makefile.write(" " + md.identifier)
            
     makefile.write("\n")
     makefile.close()

     main_makefile = open("Makefile", "w")
     main_makefile.write("SUBDIRS =")
     for pd in program_dirs:
       main_makefile.write(" " + pd[1:len(pd)])
     main_makefile.write("\n\n")
     main_makefile.write("all: debug\n")
     main_makefile.write("\n")
     main_makefile.write("debug:\n")
     main_makefile.write("\tfor dir in $(SUBDIRS); do make -C $$dir debug; done\n")
     main_makefile.write("\n")
     main_makefile.write("optimized:\n")
     main_makefile.write("\tfor dir in $(SUBDIRS); do make -C $$dir optimized; done\n")  
     main_makefile.write("\n")
     main_makefile.write("clean:\n")
     main_makefile.write("\tfor dir in $(SUBDIRS); do make -C $$dir clean; done\n")
     main_makefile.write("\trm -f " + clean_pattern + "\n")
  
     main_makefile.close()

print_intro()

modules_file = open("modules", "r")
load_modules(modules_file)
modules_file.close()

selection_file = open(sys.argv[1], "r")
load_selection(selection_file)
selection_file.close()

clean_file = open("clear", "r")
clean_pattern = clean_file.readline()
clean_file.close()

generate_rules(clean_pattern)
