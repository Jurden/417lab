// Jordan Millett

#include <iostream>     // Include the standard C++ IO library functions
#include <cstdlib>       // For exit() prototype
#include <iomanip>

using namespace std;

class Matrix41;

class Matrix44 {
	private:
		double element[4][4];
	public:
		Matrix44(void);
		Matrix44 transpose(void) const;
		friend istream& operator>> (istream& s, Matrix44& t);
		friend ostream& operator<< (ostream& s, const Matrix44& t);
		friend Matrix44 operator*(const Matrix44 &A, const Matrix44 &B); // Overriding * operator to multiply the matrices
		friend Matrix41 operator*(const Matrix44 &A, const Matrix41 &P); // Overriding * operator to multiply the 4x4 and 4x1
		Matrix44 inverse(void) const;
};

Matrix44 operator*(const Matrix44 &A, const Matrix44 &B){	// Actualy overriding of *
	Matrix44 product;	// Final value
	for(int i=0; i<4; i++){
		for(int j=0; j<4; j++){
			product.element[i][j]= //For each part of the matrix
								A.element[i][0]*B.element[0][j]+
								A.element[i][1]*B.element[1][j]+
								A.element[i][2]*B.element[2][j]+
								A.element[i][3]*B.element[3][j];
		}
	}
	return product;
}
	
Matrix44::Matrix44(void) {
	for (int i=0; i<4; i++){ 
		for (int j=0; j<4; j++) {
			if (i==j) element[i][j] = 1.;
			else      element[i][j] = 0.;
		}
	}
}

Matrix44 Matrix44::transpose(void) const {
	Matrix44 result;
	for (int i=0; i<4; i++){ 
		for (int j=0; j<4; j++){ 
			result.element[i][j] = element[j][i];
		}
	}
	return result;
}

Matrix44 Matrix44::inverse(void) const{ //Inverting matrix
	Matrix44 result;	//Final value
	Matrix44 r;			//Temp value
	for(int i=0; i<3; i++){	//For the internal 3x3
		for(int j=0; j<3; j++){
			result.element[i][j]=element[j][i];	// Transpose the values
		}
	}
	for(int i=0; i<3; i++){
		result.element[i][3]=//For the 3x1 in the upper right
								-result.element[i][0]*element[0][3]+
								-result.element[i][1]*element[1][3]+
								-result.element[i][2]*element[2][3];
							//Multiply the negative of the traspose of the 3x3 and the previous 3x1 in the upper right
	}
	for(int i=0; i<3; i++){
		result.element[3][i]=element[3][i];
		//Copy over the bottom row
	}
	return result;
}
istream& operator>> (istream& s, Matrix44& t) {
	for (int i=0; i<4; i++){ 
		for (int j=0; j<4; j++){ 
			s >> t.element[i][j];
		}
	}
	if (!s) { cerr << "Error reading Matrix from stream";  exit(0); }
	return s;
}

ostream& operator<< (ostream& s, const Matrix44& t) {
	for (int i=0; i<4; i++) {
		s <<'|';
		for (int j=0; j<4; j++){ 
				s <<setw(3)<< t.element[i][j]<<setw(3); //<< "   ";
		
		}
		s <<'|'<< endl;		
	}
	if (!s) { cerr << "Error writing Matrix to stream";  exit(0); }
	return s;
}

class Matrix41 {
	private:
		double element[4][1];
	public:
		Matrix41(void);
		Matrix41 transpose(void) const;
		friend istream& operator>> (istream& s, Matrix41& t);
		friend ostream& operator<< (ostream& s, const Matrix41& t);
		friend Matrix41 operator*(const Matrix44 &A, const Matrix41 &P);
};

Matrix41 operator*(const Matrix44 &A, const Matrix41 &P){
	int j=0;
	Matrix41 product;
	for(int i=0; i<4; i++){
			product.element[i][j]=	
						//Keeps the column of the point constant
								A.element[i][0]*P.element[0][0]+
								A.element[i][1]*P.element[1][0]+
								A.element[i][2]*P.element[2][0]+
								A.element[i][3]*P.element[3][0];
	}
	return product;
}	


Matrix41::Matrix41(void) {
	//int j=0;
	//for (int i=0; i<4; i++){ 
			//if (i==j) element[i][j] = 1.;
			//else      element[i][j] = 0.;
		
	//}
	element[0][0]=0;
	element[1][0]=0;
	element[2][0]=0;
	element[3][0]=0;
}

Matrix41 Matrix41::transpose(void) const {
	int j=0;
	Matrix41 result;
	for (int i=0; i<4; i++){ 
			result.element[i][j] = element[j][i];
	}
	return result;
}

istream& operator>> (istream& s, Matrix41& t) {
	int j=0;
	for (int i=0; i<3; i++){ //Only reads 3 values
			s >> t.element[i][j];
	}
	t.element[3][0]=1;	//Adds 4th row with value 1
	if (!s) { cerr << "Error reading Matrix from stream";  exit(0); }
	return s;
}

ostream& operator<< (ostream& s, const Matrix41& t) {
	int j=0;
	for (int i=0; i<3; i++) { // Only prints 3 rows
			s << '|' << setw(3) << t.element[i][j] << setw(3) << '|';
			s << endl;
	}
	if (!s) { cerr << "Error writing Matrix to stream";  exit(0); }
	return s;
}

void clear(void){ // Prints return to clear the terminal
	for(int j=0;j<40;j++){
		cout << endl;
	}
}
int main(int, char* []){                    // Begin the main program
	int i,x;
	Matrix44 m,n;
	Matrix41 p;
	
	while(x){
		cout << endl << "1 - Enter matrix A";
		cout << endl << "2 - Enter matrix B";
		cout << endl << "3 - Enter point P";
		cout << endl << "4 - Multiply";
		cout << endl << "5 - Transform";
		cout << endl << "6 - Inverse";
		cout << endl << "7 - Print matrices";
		cout << endl << "8 - Quit" << endl;
		cin >> i;	//Read input
		switch (i){
			case 1:
				clear();
				cout << endl << "Enter 16 elements of matrix A" << endl;
				cin >> m;	//Assign input to matrix A
				break;
			case 2:
				clear();
				cout << endl << "Enter 16 elements of matrix B" << endl;
				cin >> n;	//Assign input to matrix B
				break;
			case 3:
				clear();
				cout << endl << "Enter 3 elements of point P" << endl;
				cin >> p;	//Assign input to matrix P
				break;
			case 4:
				clear();
				cout<<endl<<"A x B"<< endl<<(m*n)<<endl;
							// Calls operator to multiply matrices	
				break;
			case 5:
				clear();
				cout<<endl<<"A x P"<< endl<<(m*p)<<endl;	
						//Calls operator for 4x4 * 4x1
				break;
			case 6:
				clear();
				cout<<endl<<"Inverse of A"<< endl<<m.inverse()<<endl;
						// Calls inverse
				break;
			case 7:
				clear();
				cout <<"Matrix A"<<endl<< m<<endl;
				cout <<"Matrix B"<<endl<< n<<endl;
				cout <<"Point P"<<endl<< p<<endl;
				break;
			case 8:
				x=0;
				break;
			default:
				cout << endl << "Invalid option" << endl;
				break;
		}
	}
}


