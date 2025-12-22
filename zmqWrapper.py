import zmq


context = zmq.Context()

# Export context for use in other modules
__all__ = ['subscribe', 'publisher', 'context']

def subscribe(topics, port, ip='127.0.0.1'):
    zmqSub = context.socket(zmq.SUB)
    zmqSub.setsockopt(zmq.CONFLATE, 1) 
    zmqSub.connect("tcp://%s:%d" % (ip,port))
    for topic in topics:
        zmqSub.setsockopt(zmq.SUBSCRIBE,topic)
    return zmqSub

def publisher(port,ip='127.0.0.1'):
    zmqPub = context.socket(zmq.PUB)
    zmqPub.setsockopt(zmq.LINGER, 0)  # Close socket immediately on context termination
    zmqPub.bind("tcp://%s:%d" % (ip,port) )
    return zmqPub
