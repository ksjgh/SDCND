#ifdef CLASSIFIER_H
#define CLASSIFIER_H
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

class GNB {
  public :
    vector<string> possible_labels = { "left","keep","right" };

      /**
    * Constructor
    */
    GNB();

      /**
   	* Destructor
   	*/
    virtual ~GNB();

    void train(vector<vector<double>> data, vector<string> labels);

    string predict(vector<double>);

    double gaussian_prob(x, mu, sig);

  private:
    vector<vector<double>> _means;
    vector<vector<double>> _stds;
    vector<double> _predict(vector<double> obs);
};

#endif
