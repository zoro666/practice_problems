"""
This is a psuedo code for performing non max suppression

Given a bounding_box, confidence_list, iou_threshold, confidence_threshold 
associate different bbox to the objects. Writing the function for association
given an iou function to calculate different ious

Input Args:
bbox_list = [bbox1, bbox2, bbox3, ...., bboxn]
confidence_list = [confidence1, confidence2, ....,confidencen]
confidence_threshold
iou_threshold

def iou(bbox1, bbox2):
    # IOU calculation logic
    return iou
"""

def non_max_suppression(bbox_list, confidence_list, iou_threshold, confidence_threshold):
    """
    Calculate non max suppression. BBox is an object consisting of bottom left and top right
    coordinates. 

    Output Args:
    out = list of BBOx representing an object
    """
    l = len(bbox_list)
    out = []
    visited = [0 for j in range(l)]
    for i in range(l):
        # Find bbox with confidence greater than threshold and the one which wasn't visited before
        if visited[i] == 0 and confidence_list[i] >= confidence_threshold:
            # Get the confidence level
            max1 = confidence_list[i]
            high_id = i
            # Tag the bbox as visited
            visited[i] = 1
            for j in range(l):
                # Find the other bboxes whose iou is greater than iou_threshold and
                # whose confidence is greater than confidence threshold
                if i != j and visited[j] == 0 and confidence_list[j] >= confidence_threshold and iou(bbox_list[i], bbox_list[j] >= iou_threshold):
                    # If you find bbox with higher confidence than use the bbox 
                    if confidence_list[j] > max1:
                        max1 = confidence_list[j]
                        high_id = j
                    # Tag the bbox as visited
                    visited[j] = 1
            # represent bbox with max confidence value as the object and suppress other
            bbox = bbox_list[high_id]
            # Append result
            out.append(bbox)
    return out