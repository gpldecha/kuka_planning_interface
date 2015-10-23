; Auto-generated. Do not edit!


(cl:in-package control_cmd_interface-srv)


;//! \htmlinclude String_cmd-request.msg.html

(cl:defclass <String_cmd-request> (roslisp-msg-protocol:ros-message)
  ((str
    :reader str
    :initarg :str
    :type cl:string
    :initform ""))
)

(cl:defclass String_cmd-request (<String_cmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <String_cmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'String_cmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_cmd_interface-srv:<String_cmd-request> is deprecated: use control_cmd_interface-srv:String_cmd-request instead.")))

(cl:ensure-generic-function 'str-val :lambda-list '(m))
(cl:defmethod str-val ((m <String_cmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_cmd_interface-srv:str-val is deprecated.  Use control_cmd_interface-srv:str instead.")
  (str m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <String_cmd-request>) ostream)
  "Serializes a message object of type '<String_cmd-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'str))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'str))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <String_cmd-request>) istream)
  "Deserializes a message object of type '<String_cmd-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'str) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'str) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<String_cmd-request>)))
  "Returns string type for a service object of type '<String_cmd-request>"
  "control_cmd_interface/String_cmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'String_cmd-request)))
  "Returns string type for a service object of type 'String_cmd-request"
  "control_cmd_interface/String_cmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<String_cmd-request>)))
  "Returns md5sum for a message object of type '<String_cmd-request>"
  "671f8e4998eaec79f1c47e339dfd527b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'String_cmd-request)))
  "Returns md5sum for a message object of type 'String_cmd-request"
  "671f8e4998eaec79f1c47e339dfd527b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<String_cmd-request>)))
  "Returns full string definition for message of type '<String_cmd-request>"
  (cl:format cl:nil "string str~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'String_cmd-request)))
  "Returns full string definition for message of type 'String_cmd-request"
  (cl:format cl:nil "string str~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <String_cmd-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'str))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <String_cmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'String_cmd-request
    (cl:cons ':str (str msg))
))
;//! \htmlinclude String_cmd-response.msg.html

(cl:defclass <String_cmd-response> (roslisp-msg-protocol:ros-message)
  ((str
    :reader str
    :initarg :str
    :type cl:string
    :initform ""))
)

(cl:defclass String_cmd-response (<String_cmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <String_cmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'String_cmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_cmd_interface-srv:<String_cmd-response> is deprecated: use control_cmd_interface-srv:String_cmd-response instead.")))

(cl:ensure-generic-function 'str-val :lambda-list '(m))
(cl:defmethod str-val ((m <String_cmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_cmd_interface-srv:str-val is deprecated.  Use control_cmd_interface-srv:str instead.")
  (str m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <String_cmd-response>) ostream)
  "Serializes a message object of type '<String_cmd-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'str))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'str))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <String_cmd-response>) istream)
  "Deserializes a message object of type '<String_cmd-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'str) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'str) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<String_cmd-response>)))
  "Returns string type for a service object of type '<String_cmd-response>"
  "control_cmd_interface/String_cmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'String_cmd-response)))
  "Returns string type for a service object of type 'String_cmd-response"
  "control_cmd_interface/String_cmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<String_cmd-response>)))
  "Returns md5sum for a message object of type '<String_cmd-response>"
  "671f8e4998eaec79f1c47e339dfd527b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'String_cmd-response)))
  "Returns md5sum for a message object of type 'String_cmd-response"
  "671f8e4998eaec79f1c47e339dfd527b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<String_cmd-response>)))
  "Returns full string definition for message of type '<String_cmd-response>"
  (cl:format cl:nil "string str~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'String_cmd-response)))
  "Returns full string definition for message of type 'String_cmd-response"
  (cl:format cl:nil "string str~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <String_cmd-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'str))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <String_cmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'String_cmd-response
    (cl:cons ':str (str msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'String_cmd)))
  'String_cmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'String_cmd)))
  'String_cmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'String_cmd)))
  "Returns string type for a service object of type '<String_cmd>"
  "control_cmd_interface/String_cmd")