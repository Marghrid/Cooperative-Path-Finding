#ifndef _PARSER_
#define _PARSER_

#include "instance.h"
#include <iostream>
#include <fstream>
#include <string>

class Parser {
private:
	std::ifstream _instance_file_stream;
	std::string   _instance_file;

public:
	Parser(std::string instance_file) {
		_instance_file = instance_file;
		_instance_file_stream.open(instance_file);
	}

	~Parser() {
		_instance_file_stream.close();
	}

	Instance parse();
};
#endif