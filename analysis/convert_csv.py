import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import csv

def connect(sqlite_file):
    conn = sqlite3.connect(sqlite_file)
    c = conn.cursor()
    return conn, c

def close(conn):
    conn.close()

def countRows(cursor, table_name, print_out=False):
    """ Returns the total number of rows in the database. """
    cursor.execute('SELECT COUNT(*) FROM {}'.format(table_name))
    count = cursor.fetchall()
    if print_out:
        print('\nTotal rows: {}'.format(count[0][0]))
    return count[0][0]

def getHeaders(cursor, table_name, print_out=False):
    """ Returns a list of tuples with column informations:
    (id, name, type, notnull, default_value, primary_key)
    """
    # Get headers from table "table_name"
    cursor.execute('PRAGMA TABLE_INFO({})'.format(table_name))
    info = cursor.fetchall()
    if print_out:
        print("\nColumn Info:\nID, Name, Type, NotNull, DefaultVal, PrimaryKey")
        for col in info:
            print(col)
    return info

def getAllElements(cursor, table_name, print_out=False):
    """ Returns a dictionary with all elements of the table database.
    """
    # Get elements from table "table_name"
    cursor.execute('SELECT * from({})'.format(table_name))
    records = cursor.fetchall()
    if print_out:
        print("\nAll elements:")
        for row in records:
            print(row)
    return records

def isTopic(cursor, topic_name, print_out=False):
    """ Returns topic_name header if it exists. If it doesn't, returns empty.
        It returns the last topic found with this name.
    """
    boolIsTopic = False
    topicFound = []

    # Get all records for 'topics'
    records = getAllElements(cursor, 'topics', print_out=False)

    # Look for specific 'topic_name' in 'records'
    for row in records:
        if(row[1] == topic_name): # 1 is 'name' TODO
            boolIsTopic = True
            topicFound = row
    if print_out:
        if boolIsTopic:
             # 1 is 'name', 0 is 'id' TODO
            print('\nTopic named', topicFound[1], ' exists at id ', topicFound[0] ,'\n')
        else:
            print('\nTopic', topic_name ,'could not be found. \n')

    return topicFound

def getAllMessagesInTopic(cursor, topic_name, print_out=False):
    """ Returns all timestamps and messages at that topic.
    There is no deserialization for the BLOB data.
    """
    count = 0
    timestamps = []
    messages = []

    # Find if topic exists and its id
    topicFound = isTopic(cursor, topic_name, print_out=False)

    # If not find return empty
    if not topicFound:
        print('Topic', topic_name ,'could not be found. \n')
    else:
        records = getAllElements(cursor, 'messages', print_out=False)

        # Look for message with the same id from the topic
        for row in records:
            if row[1] == topicFound[0]:     # 1 and 0 is 'topic_id' TODO
                count = count + 1           # count messages for this topic
                timestamps.append(row[2])   # 2 is for timestamp TODO
                messages.append(row[3])     # 3 is for all messages

        # Print
        if print_out:
            print('\nThere are ', count, 'messages in ', topicFound[1])

    return timestamps, messages

def getAllTopicsNames(cursor, print_out=False):
    """ Returns all topics names.
    """
    topicNames = []
    # Get all records for 'topics'
    records = getAllElements(cursor, 'topics', print_out=False)

    # Save all topics names
    for row in records:
        topicNames.append(row[1])  # 1 is for topic name TODO
    if print_out:
        print('\nTopics names are:')
        print(topicNames)

    return topicNames

def getAllMsgsTypes(cursor, print_out=False):
    """ Returns all messages types.
    """
    msgsTypes = []
    # Get all records for 'topics'
    records = getAllElements(cursor, 'topics', print_out=False)

    # Save all message types
    for row in records:
        msgsTypes.append(row[2])  # 2 is for message type TODO
    if print_out:
        print('\nMessages types are:')
        print(msgsTypes)

    return msgsTypes

def getMsgType(cursor, topic_name, print_out=False):
    """ Returns the message type of that specific topic.
    """
    msg_type = []
    # Get all topics names and all message types
    topic_names = getAllTopicsNames(cursor, print_out=False)
    msgs_types = getAllMsgsTypes(cursor, print_out=False)

    # look for topic at the topic_names list, and find its index
    for index, element in enumerate(topic_names):
        if element == topic_name:
            msg_type = msgs_types[index]
    if print_out:
        print('\nMessage type in', topic_name, 'is', msg_type)

    return msg_type

# ... [Your existing imports and functions]

if __name__ == "__main__":

    # Specify the path for the CSV file
    csv_file_path = '/home/akshaj/Downloads/details5.csv'

    # path to the bagfile
    bag_file = '/home/akshaj/Downloads/5hr_data_0.db3'

    # topic name
    topic_name = '/imu'

    ### connect to the database
    conn, c = connect(bag_file)

    ### get all topics names and types
    topic_names = getAllTopicsNames(c, print_out=False)
    topic_types = getAllMsgsTypes(c, print_out=False)

    # Create a map for quicker lookup
    type_map = {topic_names[i]: topic_types[i] for i in range(len(topic_types))}

    ### get all timestamps and all messages
    t, msgs = getAllMessagesInTopic(c, topic_name, print_out=True)

    # Deserialize the message
    msg_type = get_message(type_map[topic_name])
    
    # Open the CSV file for writing
    with open(csv_file_path, 'w', newline='') as csvfile:
        # Create a CSV writer
        csv_writer = csv.writer(csvfile)
        
        # Write the header row
        csv_writer.writerow(['Time_sec', 'Time_nsec', 'Orientation_x', 'Orientation_y', 'Orientation_z', 'Orientation_w',
                             'Angular_velocity_x', 'Angular_velocity_y', 'Angular_velocity_z',
                             'Linear_acceleration_x', 'Linear_acceleration_y', 'Linear_acceleration_z',
                             'Magnetic_field_x', 'Magnetic_field_y', 'Magnetic_field_z'])
        
        for timestamp, message in zip(t, msgs):
            deserialized_msg = deserialize_message(message, msg_type)

            # Extracting the required fields
            time_sec = deserialized_msg.header.stamp.sec
            time_nsec = deserialized_msg.header.stamp.nanosec
            orientation_x = deserialized_msg.imu.orientation.x
            orientation_y = deserialized_msg.imu.orientation.y
            orientation_z = deserialized_msg.imu.orientation.z
            orientation_w = deserialized_msg.imu.orientation.w
            angular_velocity_x = deserialized_msg.imu.angular_velocity.x
            angular_velocity_y = deserialized_msg.imu.angular_velocity.y
            angular_velocity_z = deserialized_msg.imu.angular_velocity.z
            linear_acceleration_x = deserialized_msg.imu.linear_acceleration.x
            linear_acceleration_y = deserialized_msg.imu.linear_acceleration.y
            linear_acceleration_z = deserialized_msg.imu.linear_acceleration.z
            magnetic_field_x = deserialized_msg.mag_field.magnetic_field.x
            magnetic_field_y = deserialized_msg.mag_field.magnetic_field.y
            magnetic_field_z = deserialized_msg.mag_field.magnetic_field.z
            
            # Write the extracted fields to the CSV
            csv_writer.writerow([time_sec, time_nsec,
                                 orientation_x, orientation_y, orientation_z, orientation_w,
                                 angular_velocity_x, angular_velocity_y, angular_velocity_z,
                                 linear_acceleration_x, linear_acceleration_y, linear_acceleration_z,
                                 magnetic_field_x, magnetic_field_y, magnetic_field_z])
    
    ### close connection to the database
    close(conn)


