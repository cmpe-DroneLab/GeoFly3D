from sqlalchemy import create_engine, Column, Integer, String, Boolean, Float, ForeignKey
from sqlalchemy.orm import relationship, backref, sessionmaker
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

from sqlalchemy import create_engine, Column, Integer, String, Boolean, Float, ForeignKey
from sqlalchemy.orm import relationship, backref
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()


class Drone(Base):
    __tablename__ = 'drones'

    drone_id = Column(Integer, primary_key=True)
    model = Column(String)
    battery_no = Column(Integer)
    flight_status = Column(String)
    gps_status = Column(Boolean)
    connection_status = Column(Boolean)
    take_off_node_id = Column(Integer, ForeignKey('nodes.node_id'))
    current_node_id = Column(Integer, ForeignKey('nodes.node_id'))
    destination_node_id = Column(Integer, ForeignKey('nodes.node_id'))
    mission_id = Column(Integer, ForeignKey('missions.mission_id'))

    take_off_node = relationship("Node", foreign_keys=[take_off_node_id], backref=backref("take_off_drones", cascade="all, delete"))
    current_node = relationship("Node", foreign_keys=[current_node_id], backref=backref("current_drones", cascade="all, delete"))
    destination_node = relationship("Node", foreign_keys=[destination_node_id], backref=backref("destination_drones", cascade="all, delete"))
    mission = relationship("Mission", back_populates="mission_drones")
    events = relationship("Event", back_populates="drone")


class Mission(Base):
    __tablename__ = 'missions'

    mission_id = Column(Integer, primary_key=True)
    creation_time = Column(String)
    last_update_time = Column(String)
    center_lat = Column(Float)
    center_lon = Column(Float)
    gcs_lat = Column(Float)
    gcs_lon = Column(Float)
    coordinates = Column(String)
    mission_status = Column(String)
    estimated_mission_time = Column(Integer)
    actual_mission_time = Column(Integer)
    required_battery_capacity = Column(Integer)
    selected_area = Column(Integer)
    scanned_area = Column(Integer)
    altitude = Column(Integer)
    gimbal_angle = Column(Integer)
    route_angle = Column(Integer)
    rotated_route_angle = Column(Integer)
    mission_drones = relationship("Drone", cascade="all, delete", back_populates="mission")
    project_folder = Column(String)
    last_visited_node_lat = Column(Float)
    last_visited_node_lon = Column(Float)


class Event(Base):
    __tablename__ = 'events'

    event_id = Column(Integer, primary_key=True)
    drone_id = Column(Integer, ForeignKey('drones.drone_id'))
    event_time = Column(String)
    event_altitude = Column(Float)
    event_latitude = Column(Float)
    event_longitude = Column(Float)
    event_type = Column(String)

    drone = relationship("Drone", back_populates="events")


class Node(Base):
    __tablename__ = 'nodes'

    node_id = Column(Integer, primary_key=True)
    latitude = Column(Float)
    longitude = Column(Float)
    altitude = Column(Float)


def get_all_missions():
    Session = sessionmaker(bind=engine)
    session = Session()
    missions = session.query(Mission).all()
    session.close()
    return missions


def get_mission_drones(mission_id):
    Session = sessionmaker(bind=engine)
    session = Session()
    drones = session.query(Drone).filter_by(mission_id=mission_id).all()
    session.close()
    return drones


# SQLite veritabanı motorunu oluşturma
engine = create_engine('sqlite:///database.db')
Base.metadata.create_all(engine)

# Session oluştur
Session = sessionmaker(bind=engine)
session = Session()
