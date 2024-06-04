from sqlalchemy import create_engine, Column, Integer, String, Boolean, Float, ForeignKey
from sqlalchemy.orm import relationship, sessionmaker
from sqlalchemy.ext.declarative import declarative_base

engine = create_engine('sqlite:///database.db')
Session = sessionmaker(bind=engine)
session = Session()
Base = declarative_base()


class Node(Base):
    __tablename__ = 'nodes'

    node_id = Column(Integer, primary_key=True)
    latitude = Column(Float)
    longitude = Column(Float)

    def add_to_db(self):
        session.add(self)
        session.commit()


class Path(Base):
    __tablename__ = 'paths'

    path_id = Column(Integer, primary_key=True)
    path_boundary = Column(String)
    vertex_count = Column(Integer)
    opt_route = Column(String)
    rot_route = Column(String)
    opt_route_length = Column(Float)
    rot_route_length = Column(Float)
    actual_flown = Column(Float, default=0.0)
    increment = Column(Float, default=0.0)

    mission_id = Column(Integer, ForeignKey('missions.mission_id'))
    last_visited_node_id = Column(Integer, ForeignKey('nodes.node_id', ondelete='CASCADE'))
    start_point_id = Column(Integer, ForeignKey('nodes.node_id', ondelete='CASCADE'))

    mission = relationship("Mission", foreign_keys=[mission_id], back_populates="mission_paths")
    last_visited_node = relationship("Node", foreign_keys=[last_visited_node_id], cascade="all, delete")
    start_point = relationship("Node", foreign_keys=[start_point_id], cascade="all, delete")

    def set_actual_flown(self, actual):
        self.actual_flown = actual
        session.add(self)
        session.commit()

    def set_increment(self, increment):
        self.increment = increment
        session.add(self)
        session.commit()

    def add_to_db(self):
        session.add(self)
        session.commit()


class Drone(Base):
    __tablename__ = 'drones'

    drone_id = Column(Integer, primary_key=True)
    model = Column(String)
    ip_address = Column(String)
    battery_no = Column(Integer)
    flight_status = Column(String)
    gps_status = Column(Boolean)
    connection_status = Column(Boolean)
    mission_id = Column(Integer, ForeignKey('missions.mission_id'))
    path_id = Column(Integer, ForeignKey('paths.path_id'))

    mission = relationship("Mission", foreign_keys=[mission_id], back_populates="mission_drones")
    path = relationship("Path", foreign_keys=[path_id], cascade="all, delete")

    def add_to_db(self):
        session.add(self)
        session.commit()


class Mission(Base):
    __tablename__ = 'missions'

    mission_id = Column(Integer, primary_key=True)
    creation_time = Column(String)
    last_update_time = Column(String)
    flight_start_time = Column(String)
    flight_finish_time = Column(String)
    center_node_id = Column(Integer, ForeignKey('nodes.node_id', ondelete='CASCADE'))
    gcs_node_id = Column(Integer, ForeignKey('nodes.node_id', ondelete='CASCADE'))
    mission_boundary = Column(String)
    mission_status = Column(String)
    estimated_mission_time = Column(Integer)
    actual_mission_time = Column(Integer)
    provided_battery_capacity = Column(Integer)
    required_battery_capacity = Column(Integer)
    selected_area = Column(Integer)
    altitude = Column(Integer)
    gimbal_angle = Column(Integer)
    route_angle = Column(Integer)
    rotated_route_angle = Column(Integer)
    project_folder = Column(String)

    center_node = relationship("Node", foreign_keys=[center_node_id], cascade="all, delete")
    gcs_node = relationship("Node", foreign_keys=[gcs_node_id], cascade="all, delete")
    mission_drones = relationship("Drone", back_populates="mission", cascade="all, delete")
    mission_paths = relationship("Path", back_populates="mission", cascade="all, delete")

    def add_to_db(self):
        session.add(self)
        session.commit()


    def set_status(self, status):
        session.query(Mission).filter_by(mission_id=self.mission_id).update({"mission_status": status})
        session.commit()

    def set_project_folder(self, folder):
        session.query(Mission).filter_by(mission_id=self.mission_id).update({"project_folder": folder})
        session.commit()

    def set_flight_start_time(self, flight_start_time):
        session.query(Mission).filter_by(mission_id=self.mission_id).update({"flight_start_time": flight_start_time})
        session.commit()


def delete_mission_by_id(mission_id):
    mission = session.query(Mission).filter_by(mission_id=mission_id).first()
    if mission:
        session.delete(mission)
        session.commit()


def get_drone_by_path_id(path_id):
    return session.query(Drone).join(Path).filter(Path.path_id == path_id).first()


def get_drone_by_id(drone_id):
    return session.query(Drone).filter_by(drone_id=drone_id).first()


def get_mission_by_id(mission_id):
    return session.query(Mission).filter_by(mission_id=mission_id).first()


def get_mission_by_drone_id(drone_id):
    return session.query(Mission).join(Drone).filter(Drone.drone_id == drone_id).first()


def get_all_missions():
    missions = session.query(Mission).all()
    return missions


Base.metadata.create_all(engine)