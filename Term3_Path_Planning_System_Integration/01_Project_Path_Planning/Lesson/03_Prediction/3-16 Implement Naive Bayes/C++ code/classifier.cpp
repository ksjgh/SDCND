#include "classifier.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>

/**
 * Initializes GNB
 */
GNB::GNB() {

}


GND::~GNB(){}

double GNB::gaussian_prob(double x, double mu, double sig){
    const double pi = 3.1415926535897;
    double num = pow((x - mu),2.0);
    double denum = 2.0*pow(sig,2);
    double norm = 1 / (sqrt(2*pi)*sig);
    return norm * exp(-num/denum);
}

void GNB::train(vector<vector<double>> data, vector<string> labels){

// reorder data by labels
  vector<vector<double>> totals_by_label;
  int num_vars = data[0].size();
  int num_labels = possible_labels.size();

  totals_by_label.resize(num_labels);
  for(int i=0; i < num_labels; i++)
    totals_by_label[i].resize(num_vars);

  // find label's index
  for(size_t i=0; i<data.size(); i++){
    vector<double> vals = data[i];
    string label = labels[i];
    int label_idx;
    for(int i=0; i < possible_labels.size(); i++){
      if(possible_labels[i] == label){
          label_idx = i;
          break;
      }
    }
  }


  for(int i=0; i < num_vars; i++)
    totals_by_label[label_idx][i].push_back(vals[i]);

  _means.resize(num_labels);
  _stds.resize(num_labels);
  for(int i=0; i < num_labels; i++){
    for(int j=0; j < num_vars; j++){
      vector<double> v = totals_by_label[i][j]
      double mean = accumulate( v.begin(),v.end(), 0.0)/v.size());
      _means[i].push_back(mean);

      vector<double> diff(v.size());
      transform(v.begin(), v.end(), diff.begin(),bind2nd(minus<double>(), mean));
      double sq_sum = inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
      double stdev = sqrt(sq_sum / v.size());
      _stds[i].push_back(stdev);
    }
  }
}

vector<double> GNB::_predict(vector<double> obs){
  vector<double> probs(possible_labels.size());

  for(int i = 0; i < possible_labels.size(); i++){
    double product = 1.0;
    for(int j =0; j < obs.size()){
      double mean = _means[i][j];
      double stdev = _stds[i][j];
      double o = obs[j];

      likelihood = gaussian_prob(o, mean, stdev);
      product *= likelihood;
    }
    probs.push_back(product);
  }

  double sum = accumulate( probs.begin(),probs.end(), 0.0)/probs.size());
  for(int i = 0; i < possible_labels.size(); i++)
    probs[i] /= sum;

  return probs;

}

string GND::predict(vector<double> sample){
  vector<double> probs = this->_predict(sample);
  int k;
  double best_p = 0.0;
  for(int i=0; i < probs.size(); i++){
    if(prob[i] > best_p){
      best_p = prob[i];
      k = i;
    }
  }
  return this->possible_labels[k];
}
