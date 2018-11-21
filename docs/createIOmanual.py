# Python script for scraping the documentation of NDPluginCVHelper.cpp to create an
# html file containing information on the inputs and outputs of each of the supported 
# wrapper funcitons
#
# Author: Jakub Wlodek
# Created on: November 21, 2018
# Copyright (c): Brookhaven National Laboratory
#

import os


# Function that removes all whitespace in a string
def remove_whitespace(line):
    line.strip()
    no_whitespace = line.replace(" ", "")
    return no_whitespace


# Function that writes base html to the manual
def write_manual_top():
    manual_file = open("manual.html", "w+")
    manual_top = open("manual_base.txt", "r+")
    line = manual_top.readline()
    while line:
        manual_file.write(line)
        line = manual_top.readline()
    manual_file.close()
    manual_top.close()


# Function that writes the closing statements to the table
def write_close_table():
    manual_file = open("manual.html", "a")
    manual_file.write("\n</tbody>\n")
    manual_file.write("</table>\n")
    manual_file.close()


#function that closes out the html
def write_manual_bottom():
    manual_file = open("manual.html", "a")
    manual_file.write("\n</body>\n")
    manual_file.write("</html>\n")
    manual_file.close()


# Function that reads the Helper lib for ADCompVision and searches the comments for Wrapper functions
# It then gets input output information and sends them to the html generator
def parse_comments_table():
    helper_file = open("../adcvApp/adcvSrc/NDPluginCVHelper.cpp", "r+")

    functions = []
    functionCounter = 0

    line = helper_file.readline()
    while line:
        if "WRAPPER" in line:
            function = []
            line_nowhitespace = remove_whitespace(line)
            function.append(line_nowhitespace.split("->")[1])
            functions.append(function)
            functionCounter = functionCounter + 1
        elif "@inCount" in line:
            line_nowhitespace = remove_whitespace(line)
            functions[functionCounter-1].append(line_nowhitespace.split("->")[1])
        elif "@inFormat" in line:
            functions[functionCounter-1].append(line.split("->")[1][1:])
        elif "@outCount" in line:
            line_nowhitespace = remove_whitespace(line)
            functions[functionCounter-1].append(line_nowhitespace.split("->")[1])
        elif "@outFormat" in line:
            functions[functionCounter-1].append(line.split("->")[1][1:])
        line = helper_file.readline()
    return functions

# Function that searches the Helper lib for WRAPPER functions and gets their descriptions
# The descriptions are then passed to the html generator
def parse_comments_descriptions():
    helper_file = open("../adcvApp/adcvSrc/NDPluginCVHelper.cpp", "r+")

    functions = []
    functionCounter = 0

    line = helper_file.readline()
    while line:
        if "WRAPPER" in line:
            function = []
            line_nowhitespace = remove_whitespace(line)
            function.append(line_nowhitespace.split("->")[1])
            description = ""
            line = helper_file.readline()
            while "@inCount" not in line and line:
                description = description + line[2:]
                line = helper_file.readline()
            function.append(description)
            functions.append(function)
        line = helper_file.readline()
    return functions
            

# Function that generates the table of I/O for html
def generate_html_table(functions):
    manual = open("manual.html", "a")
    for function in functions:
        manual.write("<tr>\n")
        for item in function:
            manual.write("<th>{}</th>\n".format(item))
        manual.write("</tr>\n")
    manual.close()

# Function that generates function descriptions from comments into HTML
def generate_html_descriptions(functions):
    manual = open("manual.html","a")
    manual.write("<h2>Function descriptions</h2>\n")
    for function in functions:
        manual.write("<h3>{}</h3>".format(function[0]))
        manual.write("<p>{}</p>".format(function[1]))
    manual.close()

# Main function
def create_manual():
    if os.path.exists("manual.html"):
        os.remove("manual.html")
    
    print("Writing base of manual")
    write_manual_top()

    print("Getting info from comments")
    functions = parse_comments_table()

    print("Generating table of I/O")
    generate_html_table(functions)

    print("Closing the table")
    write_close_table()

    print("Getting function descriptions")
    descFunctions = parse_comments_descriptions()
    

    print("Adding descriptions to html")
    generate_html_descriptions(descFunctions)

    print("Finished creating the I/O manual")
    write_manual_bottom()
    

create_manual()