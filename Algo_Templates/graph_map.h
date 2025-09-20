/***************************************************************************************************
* File:           graph_map.cpp
* Description:    This file contains the implementation of a KeyedMatrix class. This class
* provides a wrapper around an Eigen::MatrixXd, allowing elements to be
* accessed and modified using string-based keys instead of integer indices.
* It also includes a main function to demonstrate its usage.
***************************************************************************************************/
#ifndef GRAPH_MAP_CPP // Include guard
#define GRAPH_MAP_CPP

#include <Eigen/Dense>
#include <unordered_map>
#include <string>
#include <iostream>

/***************************************************************************************************
* Class:          KeyedMatrix
* Description:    Manages a matrix where elements are mapped to and accessed via string keys.
* This class encapsulates an Eigen matrix and a hash map to associate keys
* with specific (row, col) coordinates within the matrix.
***************************************************************************************************/
class KeyedMatrix 
{
private:
    Eigen::MatrixXd mat;
    std::unordered_map<std::string, std::pair<int,int>> keyToIndex;

public:
    /***********************************************************************************************
    * Function:       KeyedMatrix (Constructor)
    * Description:    Initializes the matrix with a given number of rows and columns. The matrix
    * is zero-initialized.
    * Inputs:         rows - The number of rows for the matrix.
    * cols - The number of columns for the matrix.
    * Outputs:        None.
    ***********************************************************************************************/
    KeyedMatrix(int rows, int cols) : mat(rows, cols) 
    {
        mat.setZero();
    }

    /***********************************************************************************************
    * Function:       addKey
    * Description:    Adds or updates a mapping between a string key and a specific (row, col)
    * index in the matrix. Performs bounds checking.
    * Inputs:         key - The string key to associate with the index.
    * row - The row index.
    * col - The column index.
    * Outputs:        None.
    ***********************************************************************************************/
    void addKey(const std::string& key, int row, int col) 
    {
        if (row >= mat.rows() || col >= mat.cols()) 
        {
            std::cerr << "Error: Index out of bounds when adding key '" << key << "'\n";
            return;
        }
        keyToIndex[key] = {row, col};
    }

    /***********************************************************************************************
    * Function:       setValue
    * Description:    Sets the value of a matrix element identified by its string key.
    * If the key is not found, an error is printed.
    * Inputs:         key   - The string key of the element to modify.
    * value - The new double value to set.
    * Outputs:        None.
    ***********************************************************************************************/
    void setValue(const std::string& key, double value) 
    {
        auto it = keyToIndex.find(key);
        if (it != keyToIndex.end()) 
        {
            int r = it->second.first;
            int c = it->second.second;
            mat(r, c) = value;
        } 
        else 
        {
            std::cerr << "Error: Key '" << key << "' not found\n";
        }
    }

    /***********************************************************************************************
    * Function:       getValue
    * Description:    Gets the value of a matrix element by its string key.
    * If the key is not found, a warning is printed and 0.0 is returned.
    * Inputs:         key - The string key of the element to retrieve.
    * Outputs:        A double representing the value at the key's location, or 0.0 if not found.
    ***********************************************************************************************/
    double getValue(const std::string& key) const 
    {
        auto it = keyToIndex.find(key);
        if (it != keyToIndex.end()) 
        {
            int r = it->second.first;
            int c = it->second.second;
            return mat(r, c);
        } 
        else 
        {
            std::cerr << "Warning: Key '" << key << "' not found\n";
            return 0.0;
        }
    }

    /***********************************************************************************************
    * Function:       resize
    * Description:    Resizes the underlying matrix while preserving existing data where possible.
    * It also removes any keys that now point to out-of-bounds indices.
    * Inputs:         newRows - The new number of rows.
    * newCols - The new number of columns.
    * Outputs:        None.
    ***********************************************************************************************/
    void resize(int newRows, int newCols) 
    {
        mat.conservativeResize(newRows, newCols);
        // You might want to check keys that now point outside new size:
        for (auto it = keyToIndex.begin(); it != keyToIndex.end(); ) 
        {
            int r = it->second.first;
            int c = it->second.second;
            if (r >= newRows || c >= newCols) 
            {
                std::cerr << "Warning: Removing key '" << it->first << "' due to resize out-of-bounds\n";
                it = keyToIndex.erase(it);
            } 
            else 
            {
                ++it;
            }
        }
    }

    /***********************************************************************************************
    * Function:       printMatrix
    * Description:    Prints the entire underlying Eigen matrix to the standard output.
    * Inputs:         None.
    * Outputs:        None.
    ***********************************************************************************************/
    void printMatrix() const 
    {
        std::cout << "Matrix:\n" << mat << "\n";
    }
};

// ***************************************************************************************************

/***************************************************************************************************
* Function:       main
* Description:    Entry point of the program. Demonstrates the creation and use of the
* KeyedMatrix class, including adding keys, setting values, resizing the
* matrix, and retrieving data.
***************************************************************************************************/

/*


int main() 
{
    KeyedMatrix km(3, 3);

    // Add keys mapping to matrix positions
    km.addKey("A", 0, 0);
    km.addKey("B", 1, 2);
    km.addKey("C", 2, 1);

    // Set values by key
    km.setValue("A", 5.5);
    km.setValue("B", 10.1);
    km.setValue("C", 20.2);

    // Print matrix and values
    km.printMatrix();
    std::cout << "Value at 'B': " << km.getValue("B") << "\n";

    // Resize matrix to 4x5
    km.resize(4, 5);
    std::cout << "After resizing:\n";
    km.printMatrix();

    // Add a key in new region
    km.addKey("D", 3, 4);
    km.setValue("D", 99.9);
    km.printMatrix();

    return 0;
}

*/

#endif // GRAPH_MAP_CPP