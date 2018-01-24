# Augment - C++ Source Code Augmenter
# Version: 1.1
# (C) Copyright 2010-2011 Pavel Surynek
# http://www.surynek.com
# pavel.surynek@mff.cuni.cz

import fileinput
import string
import shutil
import os


def print_header_frame(file, width, height):
  file.write("/*")
  for j in range(width-4):
    file.write("-")
  file.write("*/")
  file.write("\n")

  for i in range(height-2):
    file.write("/*")
    for j in range(width-4):
      file.write(" ")
    file.write("*/")
    file.write("\n")

  file.write("/*")
  for j in range(width-4):
    file.write("-")
  file.write("*/")
  file.write("\n")


def print_header_info_frame(file, width, height, product, product_line, copyright, copyright_line, email, email_line, filename):
  file.write("/*")
  for j in range(width-4):
    file.write("=")
  file.write("*/")
  file.write("\n")

  for i in range(height-2):
    if i == product_line:
      left = (width - len(product) - 4) / 2
      right = width - len(product) - left - 4
      file.write("/*")
      for j in range(left):
        file.write(" ")
      file.write(product)
      for j in range(right):
        file.write(" ")
      file.write("*/")
      file.write("\n")

    elif i == copyright_line:
      left = (width - len(copyright) - 4) / 2
      right = width - len(copyright) - left - 4
      file.write("/*")
      for j in range(left):
        file.write(" ")
      file.write(copyright)
      for j in range(right):
        file.write(" ")
      file.write("*/")
      file.write("\n")

    elif i == email_line:
      left = (width - len(email) - 4) / 2
      right = width - len(email) - left - 4
      file.write("/*")
      for j in range(left):
        file.write(" ")
      file.write(email)
      for j in range(right):
        file.write(" ")
      file.write("*/")
      file.write("\n")

    else:
      file.write("/*")
      for j in range(width-4):
        file.write(" ")
      file.write("*/")
      file.write("\n")

  file.write("/*")
  for j in range(width-4):
    file.write("=")
  file.write("*/")
  file.write("\n")


def print_header_info_frame_files(file, filename, width, height, version, step, product, copyright, author, email, url):
  print_header_info_frame(file, width, height, product + " " + version, 2, copyright + " " + author, 4, url + " | " + "<" + email + ">", 5, filename)

  file.write("// ")
  file.write(filename + " / " + version + "_" + step)
  file.write("\n")

  file.write("/*")
  for j in range(width-4):
    file.write("-")
  file.write("*/")
  file.write("\n")


def augment_file(input_filename, output_filename, width, height, version, step, product, copyright, author, email, url, namespace):
  input_file = open(input_filename, "r")
  output_file = open(output_filename, "w")

  phase = 1

  for line in input_file:
    if phase == 1:
      if string.find(line, "/") != 0:
        print_header_info_frame_files(output_file, input_filename, width, height, version, step, product, copyright, author, email, url)
        output_file.write(line)
        phase = 3
      elif string.find(line, "//") == 0:
        phase = 2
    elif phase == 2:
      print_header_info_frame_files(output_file, input_filename, width, height, version, step, product, copyright, author, email, url)
      phase = 3
    elif phase == 3:
      if string.find(line, "/*--") == 0:
        output_file.write("/*")
        for j in range(width-4):
          output_file.write("-")
        output_file.write("*/")
        output_file.write("\n")
      elif string.find(line, "/*==") == 0:
        output_file.write("/*")
        for j in range(width-4):
          output_file.write("=")
        output_file.write("*/")
        output_file.write("\n")
      elif string.find(line, "namespace") == 0:
        output_file.write("namespace " + namespace + "\n")
      elif string.find(line, "} // namespace") == 0:
        output_file.write("} // namespace " + namespace + "\n")
      else:
        output_file.write(line)

  input_file.close()
  output_file.close()


def is_source(filename):
  if len(filename) >= 4 and string.find(filename, ".cpp") == len(filename) - 4:
    return True
  elif len(filename) >= 2 and string.find(filename, ".h") == len(filename) - 2:
    return True
  else:
    return False


def augment_directory(indent, directory, width, height, version, step, product, copyright, author, email, url, namespace):
  listing = os.listdir(directory)
  for item in listing:
    if os.path.isfile(item):
      if is_source(item):
        print indent + "Augmenting: " + item + " ... ",
        augment_file(item, item + ".aug", width, height, version, step, product, copyright, author, email, url, namespace)
        shutil.copy(item, item + ".bak")
        shutil.move(item + ".aug", item)
        print "OK"
    elif os.path.isdir(item):
      print indent + "Subdir:" + item
      os.chdir(item)
      augment_directory(indent + "  ", ".", width, height, version, step, product, copyright, author, email, url, namespace)
      os.chdir("..")


def print_intro():
  print "Augment 1.1 - C++ Source Code Augmenter"
  print "(C) Copyright 2010-2011 Pavel Surynek"
  print "---------------------------------------"


print_intro()

version_file = open("version", "r")
version = version_file.readline()
version_file.close()

step_file = open("step", "r")
step = step_file.readline()
step_file.close()

product_file = open("product", "r")
product = product_file.readline()
product_file.close()

copyright_file = open("copyright", "r")
copyright = copyright_file.readline()
copyright_file.close()

author_file = open("author", "r")
author = author_file.readline()
author_file.close()

email_file = open("email", "r")
email = email_file.readline()
email_file.close()

url_file = open("url", "r")
url = url_file.readline()
url_file.close()

namespace_file = open("namespace", "r")
namespace = namespace_file.readline()
namespace_file.close()

augment_directory("", ".", 80, 10, version, step, product, copyright, author, email, url, namespace)

