#include<string>
#include"json.h"
#include"qnode.hpp"
using namespace std;


void read_json(vector<can_signal> &signal_list);
void print_signal(can_signal signal);
int value_16to10(std::string value);
void read_signal_name_list(const string file, vector<string> &name_list);
void signal_select(const vector<can_signal> &all_signal_list, vector<can_signal> &select_signal_list, const vector<string> &name_list);

