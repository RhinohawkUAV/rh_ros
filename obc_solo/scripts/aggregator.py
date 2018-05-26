#
# Aggregation of changing ROS topic values over time. Supports the following use cases: 
# 
# 1) Latching infrequent values (with expiration)
# 2) Aggregating values over a moving window, with tolerance
# 

import rospy

class LatchedValue:
    def __init__(self, topic, max_age, value):
        self.topic = topic
        self.max_age = max_age
        self.value = value
        self.time_acquired = rospy.get_rostime()


class LatchMap:
    """ Keep track of latched values for a set of topics. Each latched value
        is optionally defined to expire after a set time. 
    """

    def __init__(self):
        self.values = {}

    def latch_value(self, topic, value, max_age=None):
        self.values[topic] = LatchedValue(topic, max_age, value)

    def get_value(self, topic):
        if topic in self.values:
            lvalue = self.values[topic]
            
            if not lvalue.time_acquired:
                # No expiration
                return lvalue.value
            
            d = lvalue.time_acquired - rospy.Time.now()
            if d.secs > lvalue.max_age:
                rospy.loginfo("Latched value too old (%d secs)", d.secs)
                return None
            
            return lvalue.value
        
        return None


from sklearn.cluster import DBSCAN
import numpy as np

def centroid(arr):
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    return sum_x / length, sum_y / length

class SpatialAggregation:

    def __init__(self, values=np.array([])):
        self.values = values

    def add_value(self, coord):
 
        # Add value
        self.values.append(coord)

        # Rerun clustering
        db = DBSCAN(eps=0.2, min_samples=3).fit(self.values)
        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True
        labels = db.labels_

        # Number of clusters in labels, ignoring noise if present.
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

        if n_clusters_>0:
            self.cluster_sizes = np.array([len(np.nonzero(labels == i)[0]) for i in range(n_clusters_)])
            k = cluster_sizes.argmax(axis=0)

            self.centroids = [centroid(X[(labels == i)]) for i in range(n_clusters_)]
            self.mean_dists = [np.mean([np.sqrt((x-centroids[i][0])**2+(y-centroids[i][1])**2) for (x, y) in X[labels == i]]) for i in range(n_clusters_)]

            for i, c in enumerate(centroids):
                print("Cluster %d - Centroid: (%2.4f,%2.4f) - Mean Distance: %2.4f" % (i,c[0],c[1],mean_dists[i]))   
            self.marker = self.centroids[k] 
        


