#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include <eigen3/Eigen/QR>
#include <cmath>
#include <limits>

#include <iostream>
#include "lqr.h"

using namespace Eigen;
using namespace std;


static bool moreNeg(ArrayXi& plist, int j){
	ArrayXi endPlist = plist.tail(plist.size()-j);
	for (int i = 0; i<endPlist.size(); i++ ){
		if (endPlist(i) < 0){
			return true;
		}
	}
	return false;
}


static MatrixPair swap(MatrixXf& U, MatrixXf&S, ArrayXi& v, ArrayXi& w){
	/*
	Swapping method attributed to Bai & Demmel : 
	https://pdfs.semanticscholar.org/40cb/1062f40805ba0299427ea3f9bf668732e098.pdf
	*/

	int p = v.size(); 
	int q = w.size();
	
	MatrixXf A11 = S.block(v(0),v(0), p, p);
	MatrixXf A22 = S.block(w(0),w(0), q, q);
	MatrixXf A12 = S.block(v(0), w(0), p, q);

	MatrixXf Ip(p,p);
	Ip.setIdentity(p, p);
	MatrixXf Iq(q,q);
	Iq.setIdentity(q,q);

	MatrixXf K = kroneckerProduct(Iq, A11) - kroneckerProduct(A22.transpose(), Ip);

	// determine gamma
	FullPivLU<MatrixXf> lu(K);
	MatrixXf H = lu.matrixLU().triangularView<Upper>();
	float gamma = H.diagonal().cwiseAbs().minCoeff();

	// solve Kx = gamma*b, where x=vec(X) and b=vec(A12)
	VectorXf b(v.size() * w.size());
	for (int j=0; j<q; j++){
		for (int k=0; k<p; k++){
			b(p*j + k) = A12(k, j);
		}
	}

	VectorXf x = K.fullPivLu().solve(gamma*b);

	//"unvectorize" x into matrix X
	MatrixXf X(p,q);
	for (int l=0; l<q; l++){
		X.col(l) << x.segment(l*p, p);
	}

	// Now use QR factorization to find Q s.t. [X; \gamma*I] = Q*[R; 0]; (the R isn't used)
	MatrixXf M_qr(p+q, q);
	M_qr << -X, gamma*Iq;
	HouseholderQR<MatrixXf> qr(M_qr);
	MatrixXf Q = qr.householderQ();

	// apply transformation Q to input  U and S to apply swap
	/*S(:,[v,w]) = S(:,[v,w])*Q;
  S([v,w],:) = Q'*S([v,w],:); 
  U(:,[v,w]) = U(:,[v,w])*Q;
  */
	S.block(0, v(0), S.rows(), v.size()+w.size()) = S.block(0, v(0), S.rows(), v.size()+w.size()) * Q;
	S.block(v(0), 0, v.size()+w.size(), S.rows()) = Q.transpose() * S.block(v(0), 0, v.size()+w.size(), S.rows());
	U.block(0, v(0), S.rows(), v.size()+w.size()) = U.block(0, v(0), S.rows(), v.size()+w.size()) * Q;

	//cout << "S: \n" << S << endl;
	//cout << "U: \n" << U << endl;


	/*
	FullPivLU<MatrixXf> lu(K);
	MatrixXf L = lu.matrixLU().triangularView<StrictlyLower>();
	L = L + MatrixXf::Identity(L.rows(), L.cols());
	MatrixXf H = lu.matrixLU().triangularView<Upper>();
	MatrixXf P = lu.permutationP();
	MatrixXf Q = lu.permutationQ();

	cout << "LU: \n " << lu.matrixLU() <<endl;
	cout << "L: \n" << L << endl;
	cout << "H: \n" << H << endl;
	cout << "reconstruct: \n" << P.inverse()*L*H*Q.inverse() << endl;

	float e = H.diagonal().cwiseAbs().minCoeff();
	for (int k=0; k<p*q-2; k++){
		r
	}
	*/
	MatrixPair US_new;
	US_new.U = U;
	US_new.S = S;

	return US_new;
}


Eigen::MatrixXf lqr(MatrixXf& A, MatrixXf& B, MatrixXf& Q_x, MatrixXf& R_u){
	
	// construct M = [-A, -B*R_u^{-1}*B^T;
	//				  -Q_x, -A^T];
	MatrixXf M(A.rows()+Q_x.rows(), A.cols()+B.rows());
	M << A, -B*R_u.inverse()*B.transpose(),
		-Q_x, -A.transpose();

	//cout << "M = \n" << M << endl;
	 
	RealSchur<MatrixXf> schur(M);
	MatrixXf q = schur.matrixU();
	MatrixXf r = schur.matrixT();

	cout << "q = \n" << q << endl;
	cout << "r = \n" << r << endl;

	float eps = numeric_limits<float>::epsilon();

	// create s-vector (list of locations of first row of 1x1 or 2x2 matrices, plus an extra one at the end)    
    ArrayXf absSubDiag = r.diagonal(-1).cwiseAbs();
    ArrayXi s(r.rows()+1);
    s.setZero();
	s(0) = 0;
	int j = 1; 
	int i = 0;  
    while (i<absSubDiag.size()){
    	if (absSubDiag(i) > 100*eps){
    		// skip if second row of a 2x2 block
    		if (i == absSubDiag.size()-1){
    			s(j) = i+2;
    			j++;
    		}
    	}
    	else{
    		s(j) = i+1;
    		j++;
    		if (i == absSubDiag.size()-1){
    			s(j) = i+2;
    			j++;
    		}
    	}
    	i++;
    }
    ArrayXi tmps = s.head(j);
	s = tmps;
    cout << "s: \n" << s << endl;

    // create plist (+1 for positive, -1 for negative real part of eigval)
    ArrayXi plist(s.size()-1);
    int N_pos = 0;
    for (int k = 0; k<s.size()-1; k++){
    	int sk = s(k);
    	// r(sk,sk) being pos or neg implies real part of eigval being pos or neg
    	if ( (s(k+1)-s(k) == 1 && r(sk,sk) > 0) || (s(k+1)-s(k) == 2 && r(sk,sk)+r(s(k)+1,s(k)+1) > 0)){
    		plist(k) = 1; // note plist(k) can be made to be actual eigval for debugging purposes
    		N_pos++;
    	}
    	else if ( (s(k+1)-s(k) == 1 && r(sk,sk) < 0) || (s(k+1)-s(k) == 2 && r(sk,sk)+r(s(k)+1,s(k)+1) < 0)){
    		plist(k) = -1;
    	}
    	else{
    		plist(k) = 0;
    	}
    }

    //cout << "r: \n" << r << endl;
    //cout << "plist: \n" << plist << endl;

    /*
    ArrayXi vtest(2), wtest(2); 
    vtest << 2,3; 
    wtest << 4,5;
	*/
    /*
    //for testing only!!
    ArrayXi vtest(1), wtest(2); 
    vtest << 2; 
    wtest << 3,4;

    cout << "q: \n" << q << endl;
    cout << "r: \n" << r << endl;
    MatrixPair US = swap(q,r,vtest,wtest);
    cout << "MatrixPair.U: \n" << US.U <<endl;
    cout << "MatrixPair.S: \n" << US.S <<endl;
    */

	cout << "r before swap: \n" << r << endl;


    // swap to arrange blocks with negative eigenvalues in upper and positive in lower
    int n = 0;
    i = 0;
    int countSwaps = 0;
    ArrayXi v;
    ArrayXi w;
    int vSize;
    int wSize;
    while (n < N_pos){
    	if (plist(i) > 0){
    		j = i;
    		// keep pushing element j back while there are negative elements after element j
    		while ((j < plist.size()) && (moreNeg(plist, j))){
    			//if (s(j) == s(j+1)-1){
    			//	v = s(j);
    			//}
    			vSize = s(j+1)-s(j);
    			v.setLinSpaced(vSize, s(j), s(j+1)-1);
    			wSize = s(j+2)-s(j+1);
    			w.setLinSpaced(wSize, s(j+1), s(j+2)-1);
    			
    			MatrixPair US = swap(q,r,v,w);
    			q = US.U;
    			r = US.S;

    			s(j+1) = s(j)+s(j+2)-s(j+1); // update s to reflect swap of 2x2 with 1x1

    			// update plist to reflect swap
    			int temp = plist(j+1);
    			plist(j+1) = plist(j);
    			plist(j) = temp;

    			j++;
    			countSwaps++;
    		}
    		n++;
    		i = 0;
    	}
    	else{
    		i++;
    	}

    }

    cout << "# of swaps: \n" << countSwaps << endl;
    
    MatrixXf Q = q;
    MatrixXf R = r; // technically R is never used later...

    //lastly, zero out lower triangular portion below the block diagonal of R and 
    
    for (int i=2; i<R.rows(); i++){
   		for (int j=0; j<i-1; j++){
    		R(i,j) = 0;
    	}
    }
    for (int k=1; k<s.size()-1; k++){
    	R(s(k), s(k)-1) = 0;
    }
    

    //cout << "Q: \n" << Q <<endl;
    cout << "R: \n" << R << endl;
	
    // calculate LQR gain
    int numNegEig = 0;
    for (int k=0; k<r.diagonal().size(); k++){
    	if (r.diagonal()(k)<0){
    		numNegEig++;
    	}
    }
    MatrixXf P = Q.block(numNegEig, 0, Q.rows()-numNegEig, numNegEig) * (Q.block(0, 0, numNegEig, numNegEig).inverse());
    cout << "P: \n " << P << endl;
    MatrixXf K = R_u.inverse() * (B.transpose()*P);

	return K;
}



// Questions: pass by reference MatrixXf will make it quicker?

//solved segmentation error: 11 by doing ArrayXi v(2) instead of ArrayXi v


