import matplotlib.pyplot as plt

class Point:
	def __init__(self, x, y):
		self.x = x
		self.y = y

	def dist(self, q):
		dist = ((self.x-q.x)**2 + (self.y-q.y)**2)**0.5
		return dist

	def slope(self, q):
		return (q.y-self.y)/(q.x-self.x)

	def add(self, b):
		if not isinstance(b, Point):
			b = Point(b[0], b[1])

		return Point(self.x+b.x, self.y+b.y)

	def sub(self, b):
		if not isinstance(b, Point):
			b = Point(b[0], b[1])

		return Point(self.x-b.x, self.y-b.y)

	def incr_slope(self, a, b, s):
		x1 = self.x + a*s
		y1 = self.y + b*s
		return Point(x1,y1)	

	def incr_point(self, q, s):
		norm = self.dist(q)
		a, b = (q.x-self.x), (q.y-self.y)
		norm = (a**2 + b**2)**0.5
		return self.incr_slope(a/norm, b/norm, s)

	def equal(self, q):
		if((self.x==q.x) and (self.y==q.y)):
			return True
		else:
			return False

def print_path(path):
	# print(path)
	p = []
	for i in path:
		p.append((i.x, i.y))
	return p

def computeLineThroughTwoPoints(p1, p2):
	
	d = p2.dist(p1)
	a = (p2.y-p1.y)/d
	b = (p1.x-p2.x)/d
	c = (p2.x*p1.y-p1.x*p2.y)/d
	return a,b,c

def computeDistancePointToLine(q, p1, p2):

	a,b,c = computeLineThroughTwoPoints(p1, p2)
	dist = abs(a*q.x + b*q.y + c)
	return dist

def computeDistancePointToSegment(q, p1, p2):

	a,b,c = computeLineThroughTwoPoints(p1, p2)
	d = p2.dist(p1)
	d1 = q.dist(p1)
	d2 = q.dist(p2)
	
	# (-b,a) is along the line
	
	dot_prod = -b*(q.x-p1.x) + a*(q.y-p1.y)
	m = dot_prod/d

	if m <= 0:
		return d1, 1
	elif m >= 1:
		return d2, 2
	else:
		return computeDistancePointToLine(q, p1, p2), 0

def conv_arr(arr):
	
	arr1 = []
	for i in range(len(arr)):
		arr1.append(Point(arr[i][0], arr[i][1]))
	return arr1

def computeDistancePointToPolygon(arr, q):

	if not isinstance(arr[0], Point):
		arr = conv_arr(arr)

	n = len(arr)
	d_min = 1e6
	for i in range(n):
		d, w = computeDistancePointToSegment(q, arr[i%n], arr[(i+1)%n])
		if d < d_min:
			d_min = d
			w_0 = w
			idx = i

	return d_min, w_0, idx

def misc_area(arr):

	if not isinstance(arr[0], Point):
		arr = conv_arr(arr)

	n = len(arr)
	summ = 0
	for i in range(n):
		summ += (arr[i%n].x - arr[(i+1)%n].x)*(arr[i%n].y + arr[(i+1)%n].y)
	if summ > 0:
		return -1
	else:
		return 1

def computeTangentVectorToPolygon(arr, q):

	if not isinstance(arr[0], Point):
		arr = conv_arr(arr)

	n = len(arr)
	d_min, w_0, idx = computeDistancePointToPolygon(arr, q)
	if w_0 == 0:
		b = misc_area(arr)
		norm = arr[idx%n].dist(arr[(idx+1)%n])
		return b*(arr[idx%n].x - arr[(idx+1)%n].x)/norm, b*(arr[idx%n].y - arr[(idx+1)%n].y)/norm
	elif w_0 == 1:
		a, b, _ = computeLineThroughTwoPoints(q, arr[idx%n])
		return a - 0.3*b, b + 0.3*a
	else:
		a, b, _ = computeLineThroughTwoPoints(q, arr[(idx+1)%n])
		return a - 0.3*b, b + 0.3*a

def poly_centroid(arr):
	if not isinstance(arr[0], Point):
		arr = conv_arr(arr)
	X, Y = 0, 0
	for i in arr:
		X+=i.x
		Y+=i.y
	return Point(X/len(arr),Y//len(arr)) 

if __name__ == '__main__':

	# p1 = Point(0,1)
	# p2 = Point(1,0)

	# print(computeLineThroughTwoPoints(p1,p2))
	# print(computeDistancePointToLine(Point(0,0),p1,p2))
	# print(computeDistancePointToLine(Point(1,-1),p1,p2))	
	# print(computeDistancePointToLine(Point(1,0),p1,p2))
	# print(computeDistancePointToLine(Point(2,-1),p1,p2))	
	# print("@@@@@@@@@@@@@@@@@@@@@")

	# print(computeDistancePointToSegment(Point(0,0),p1,p2))
	# print(computeDistancePointToSegment(Point(1,-1),p1,p2))	
	# print(computeDistancePointToSegment(Point(1,0),p1,p2))
	# print(computeDistancePointToSegment(Point(2,-1),p1,p2))
	# print("@@@@@@@@@@@@@@@@@@@@@")

	# sq = [[0,0],[0,1],[1,1],[1,0]]
	# sq = [[2, 3], [4, 1], [5, 2]]
	# sq = [[1, 2], [1, 0], [3, 0]]
	# print(computeDistancePointToPolygon(sq, Point(1.0134,0.4949)))
	# print(computeTangentVectorToPolygon(sq, Point(1.0134,0.4949)))
	# print(computeDistancePointToPolygon(sq, Point(2,2)))
	# print(computeTangentVectorToPolygon(sq, Point(2,2)))
	# print(computeDistancePointToPolygon(sq, Point(2,1)))	
	# print(computeTangentVectorToPolygon(sq, Point(2,1)))
	# print(computeDistancePointToPolygon(sq, Point(0.5,1.5)))
	# print(computeTangentVectorToPolygon(sq, Point(0.5,1.5)))
	obstacles_list = [[[1,2],[1,0],[3,0]], [[2,3],[4,1],[5,2]]]
	t1 = plt.Polygon(obstacles_list[0], color="red", fill=False)
	plt.gca().add_patch(t1)

	t2 = plt.Polygon(obstacles_list[1], color="red", fill=False)
	plt.gca().add_patch(t2)

	X, Y = [0.9432422182837985, 0.8998786043256135, 2.552340474771388, 2.576777479224643], [0.5659453309702791, 0.5993581731242988, 0.543661871132853, 0.5675924021882496]
	plt.scatter(X, Y, c="blue")
	plt.show()