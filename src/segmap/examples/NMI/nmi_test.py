
import numpy as np
import cv2


def NMI(img_a, raw_img_b):
	img_a = cv2.cvtColor(img_a, cv2.COLOR_BGR2GRAY)
	img_b = cv2.cvtColor(raw_img_b, cv2.COLOR_BGR2GRAY)	
	img_b = cv2.resize(img_b, (img_a.shape[1], img_a.shape[0]))

	cv2.imshow("img_a", img_a)
	cv2.imshow("img_b", img_b)

	linear_a = np.ravel(img_a)
	linear_b = np.ravel(img_b)
	
	nbins = 255
	hist_a = np.zeros(nbins)
	hist_b = np.zeros(nbins)	
	hist_ab = np.zeros((nbins, nbins))
	step = 255 / (nbins - 1)
	
	for i in range(len(linear_a)):
		pa = int(linear_a[i] / step)
		hist_a[pa] += 1
		pb = int(linear_b[i] / step)
		hist_b[pb] += 1
		hist_ab[pa, pb] += 1

	prob_a = hist_a / np.sum(hist_a)
	prob_b = hist_b / np.sum(hist_b)	
	prob_ab = hist_ab / np.sum(hist_ab)
	
	for i in range(len(hist_a)):
		print("bin %d: %d %d prob: %.2f %.2f" % (i, hist_a[i], hist_b[i], prob_a[i], prob_b[i]))
	
	entropy_a = np.sum([p * np.log(p) for p in prob_a if p > 0.0])
	entropy_b = np.sum([p * np.log(p) for p in prob_b if p > 0.0])
	entropy_ab = np.sum([p * np.log(p) for p in np.ravel(prob_ab) if p > 0.0])		
	
	print("entropy_a: %.2lf" % entropy_a)
	print("entropy_b: %.2lf" % entropy_b)
	print("entropy_ab: %.2lf" % entropy_ab)		
	
	nmi = (entropy_a + entropy_b) / entropy_ab
    
    # Entropy Correlation Coefficient (ECC) see "http://ijsetr.org/wp-content/uploads/2016/05/IJSETR-VOL-5-ISSUE-5-1700-1703.pdf"
	ecc = 2.0 - (2 * entropy_ab) / (entropy_a + entropy_b)
	
	#print(hist_a)
	#print(hist_b)	
	#print(hist_ab)
	#cv2.imshow("hist", 255 * (hist_ab / np.max(hist_ab)))
	#cv2.waitKey(-1)
	print("nmi:", nmi)
	print("ecc:", ecc)
	
	return ecc


if __name__ == "__main__":
	paths = ["b1.png", "b12.png", "b13.png", "b4.png", "b4.5.png"] #, "b2.png", "b3.png", "b5.png", "b6.png"]
	imgs = [cv2.imread(p) for p in paths]

	#NMI(cv2.imread("b1.png"), cv2.imread("b1.png"))
	#cv2.waitKey(-1)
	
	for i in range(len(imgs)):
		for j in range(i, len(imgs)):
			print("ECC %d %d: %lf\n" % (i, j, NMI(imgs[i], imgs[j])))
			cv2.waitKey(-1)

