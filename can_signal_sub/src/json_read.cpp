#include<iostream>
#include<fstream>
#include<sstream>
#include"../include/can_signal_sub/json_tool.h"
#include<math.h>
using namespace std;


int value_16to10(string value)
{
	int i = 0;
	int count = value.length();
	int result = 0;
	for (i = count - 1; i >= 0; i--)
	{
		if (value[i] >= '0'&&value[i] <= '9')
		{
			result += (value[i] - 48)*pow(16, count - i - 1);
		}
		else if (value[i] >= 'a'&&value[i] <= 'f')
		{
			result += (value[i] - 87)*pow(16, count - i - 1);
		}
	}
	return result;
}

void read_json(vector<can_signal> &signal_list)
{
	Json::Reader reader;
	Json::Value root;
    	ifstream in("/home/xinyh/TJURacing/TJURacing_Autonomous_System/ros/src/drivers/can_comms/can_signal_sub/config/TR23_can_config.json");

	if (!in.is_open())
	{
		cout << "Error opening file!" << endl;
		return;
	}

	int signal_num = 0;
	if (reader.parse(in, root))
	{
		for (int i = 0; i < root["can_config"].size(); i++)
		{
			for (int j = 0; j < root["can_config"][i]["signals"].size(); j++)
			{
				can_signal signal;

				signal.message_name = root["can_config"][i]["message_name"].asString();
				signal.message_length = root["can_config"][i]["message_length"].asInt();
				
                		string id_str = root["can_config"][i]["message_id"].asString().substr(2);
				signal.ID = value_16to10(id_str);
                		signal.message_format = root["can_config"][i]["format"].asString();

				signal.signal_name = root["can_config"][i]["signals"][j]["signal_name"].asString();
				signal.start_byte = root["can_config"][i]["signals"][j]["start_byte"].asInt();
				signal.start_bit = root["can_config"][i]["signals"][j]["start_bit"].asInt();
				signal.signal_length = root["can_config"][i]["signals"][j]["signal_length"].asInt();
				signal.data_type = root["can_config"][i]["signals"][j]["data_type"].asString();
				signal.factor = root["can_config"][i]["signals"][j]["factor"].asFloat();
				signal.bias = root["can_config"][i]["signals"][j]["bias"].asFloat();
				signal.maximum = root["can_config"][i]["signals"][j]["maximum"].asFloat();
				signal.minimum = root["can_config"][i]["signals"][j]["minimum"].asFloat();
				signal.unit = root["can_config"][i]["signals"][j]["unit"].asString();

				signal_list.push_back(signal);
			}
		}

		// //可对信号按名称排序
		// can_signal temp;
		// for(int i=1;i<signal_num;i++)
		// {
		// 	for(int j=0;j<signal_num-i;j++)
		// 	{
		// 		if(signal_list[j].signal_name.front()>signal_list[j+1].signal_name.front())
		// 		{
		// 		temp = signal_list[j];
		// 		signal_list[j] = signal_list[j+1];
		// 		signal_list[j+1] = temp;
		// 		}
		// 	}
		// }
	}
	else
	{
		cout << "json_config文件读取失败!" << endl;
		return;
	}
	return;
}

void  read_signal_name_list(const string file, vector<string> &name_list)
{
    ifstream infile;
    infile.open(file);
    if(!infile.is_open())
    {
        cout<<"error open engien config file!"<<endl;
        return;
    }
    string name;
    while(getline(infile, name))
		name_list.push_back(name);
}

void signal_select(const vector<can_signal> &all_signal_list, vector<can_signal> &select_signal_list, const vector<string> &name_list)
{
	for(int i=0;i<name_list.size();i++)
	{
		can_signal signal;
		signal.signal_name = name_list[i];
		for(int j=0;j<all_signal_list.size();j++)
		{
			if(all_signal_list[j].signal_name!=name_list[i])
				continue;
			else
			{
				signal.message_name = all_signal_list[j].message_name;
				signal.message_length = all_signal_list[j].message_length;
				signal.ID = all_signal_list[j].ID;
				signal.message_format = all_signal_list[j].message_format;

				signal.signal_name = all_signal_list[j].signal_name;
				signal.start_byte = all_signal_list[j].start_byte;
				signal.start_bit = all_signal_list[j].start_bit;
				signal.signal_length = all_signal_list[j].signal_length;
				signal.data_type = all_signal_list[j].data_type;
				signal.factor = all_signal_list[j].factor;
				signal.bias = all_signal_list[j].bias;
				signal.maximum = all_signal_list[j].maximum;
				signal.minimum = all_signal_list[j].minimum;
				signal.unit = all_signal_list[j].unit;
				uint64_t mask = 0;
				if(signal.data_type == "signed")
				{
					for(uint8_t index = 63; index >= signal.signal_length; index --)
					{
						mask = mask + ((uint64_t)1 << index);
					}
				}
				signal.mask = mask;

				select_signal_list.push_back(signal);
			}
		}
	}
}

void print_signal(can_signal signal)
{
	cout << "---------------------------------------" << endl;
	cout << "message_name:" << signal.message_name << endl;
	cout << "message_id:" << signal.ID << endl;
    	cout << "message_format:" << signal.message_format << endl;
	cout << "message_length:" << (int)signal.message_length << endl;
	cout << "signal_name:" << signal.signal_name << endl;
	cout << "start_byte:" << (int)signal.start_byte << endl;
	cout << "start_bit:" << (int)signal.start_bit << endl;
	cout << "signal_length:" << (int)signal.signal_length << endl;
	cout << "data_type:" << signal.data_type << endl;
	cout << "factor:" << signal.factor << endl;
	cout << "bias:" << signal.bias << endl;
	cout << "maximum:" << signal.minimum << endl;
	cout << "minimum:" << signal.maximum << endl;
	cout << "unit:" << signal.unit << endl;
	cout << "---------------------------------------" << endl;
}

