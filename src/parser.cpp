#include "parser.h"


using namespace std;

std::string subwords[TOT_COORD];
std::string light;


// splits the string into different substrings using the given delimiter
void adv_tokenizer(std::string s, char del)
{
    stringstream ss(s);
    std::string word;
    int i = 0;
    while (!ss.eof()) {

        getline(ss, word, del);
        subwords[i] = word;

        // UNCOMMENT the block if genetic_light algorithm is running
        /*
        // get the light intensity
        if(i == 9)
        {
            std::string sub;
            sub = word;
            stringstream ss(sub);
            std::string word1;
            getline(ss, word1, ']');
            light = word1;
                
        }
        */
        i += 1;
    }
}

/*
further works to extract only the float data.
flag = true then store the first splitted substring
flag = false then store the last splitted substring
*/
void adv_tokenizer1(string subs, char del, int k, bool flag)
{
    stringstream ss(subs);
    std::string word;

    int i = 0;
    while (!ss.eof()) {

        getline(ss, word, del);
        if( (i == 0) && (flag == true))
        {
            subwords[k] = word;
        }

        else if((flag == false))
        {
            subwords[k] = word;
        }
        i += 1;
    }
}

/*
Takes a string in the form of [(x1, y1, yaw1), (x2, y2, yaw2), (x3, y3, yaw3)] and returns the
float type obstacle positions
*/
float* string_parser(std::string input)
{
    GlobalVariables *gv=GlobalVariables::get_instance();
    
    //string b = "[(84.05, -14.48, 0.2), (-71.1, 84.75, -2.76), (97.07, 10.06, 3.07)]";
    // obstacle position array storing the x, y and rotation of all 3 obstacle
    bool flag = false;
    static float obs_pos[TOT_COORD];
    adv_tokenizer(input, ',');

    for(int k = 0; k < TOT_COORD; k++)
    {
        if(k % NUM_OBS == 0)
        {
            // x is the second part of the splitted string.Hence, flag = false
            adv_tokenizer1(subwords[k], '(', k, false);
        }

        else if(k % NUM_OBS == 2)
        {
            // yaw is the first part of the splitted string.Hence, flag = true
            adv_tokenizer1(subwords[k], ')', k, true);
        }

        else
        {
            // y coordinate is already in desired format
            subwords[k] = subwords[k];
        }

    }

    // converting strings to float and storing in an array
    for(int l = 0; l < TOT_COORD; l++)
    {

        obs_pos[l] = std::stof(subwords[l]);

    }

    //gv -> light_intensity = std::stof(light);

    return obs_pos;

}



