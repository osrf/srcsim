#!/usr/bin/env ruby

require 'nokogiri'

class Vector
  def initialize
    @x = 0
    @y = 0
    @z = 0
  end

  def set(x, y, z)
    @x = x
    @y = y
    @z = z
  end

  def distance(pt)
    return Math.sqrt((@x-pt.x) * (@x-pt.x) +
                     (@y-pt.y) * (@x-pt.y) +
                     (@z-pt.z) * (@x-pt.z))
  end

  attr_accessor :x
  attr_accessor :y
  attr_accessor :z
end

class Quaternion
  def initialize
    @x = 0
    @y = 0
    @z = 0
    @w = 1
  end

  def set(roll, pitch, yaw)
    phi, the, psi = 0.0

    phi = roll / 2.0
    the = pitch / 2.0
    psi = yaw / 2.0

    @w = Math.cos(phi) * Math.cos(the) * Math.cos(psi) +
      Math.sin(phi) * Math.sin(the) * Math.sin(psi)
    @x = Math.sin(phi) * Math.cos(the) * Math.cos(psi) -
      Math.cos(phi) * Math.sin(the) * Math.sin(psi)
    @y = Math.cos(phi) * Math.sin(the) * Math.cos(psi) +
      Math.sin(phi) * Math.cos(the) * Math.sin(psi)
    @z = Math.cos(phi) * Math.cos(the) * Math.sin(psi) -
      Math.sin(phi) * Math.sin(the) * Math.cos(psi)
    normalize()
  end

  def *(other)
    result = Quaternion.new
    result.w = @w*other.w-@x*other.x-@y*other.y-@z*other.z
    result.x = @w*other.x+@x*other.w+@y*other.z-@z*other.y
    result.y = @w*other.y-@x*other.z+@y*other.w+@z*other.x
    result.z = @w*other.z+@x*other.y-@y*other.x+@z*other.w
    return result
  end

  def inverse
    s = 0;
    q = Quaternion.new
    q.w = @w
    q.x = @x
    q.y = @y
    q.z = @z

    s = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

    if s == 0
      q.w = 1.0
      q.x = 0.0
      q.y = 0.0
      q.z = 0.0
    else
      q.w =  q.w / s
      q.x = -q.x / s
      q.y = -q.y / s
      q.z = -q.z / s
    end

    return q
  end

  def normalize
    s = 0.0;

    s = Math.sqrt(@w * @w + @x * @x + @y * @y + @z * @z);

    if s == 0
      @w = 1.0;
      @x = 0.0;
      @y = 0.0;
      @z = 0.0;
    else
      @w /= s;
      @x /= s;
      @y /= s;
      @z /= s;
    end
  end

  attr_accessor :x
  attr_accessor :y
  attr_accessor :z
  attr_accessor :w
end

class Pose
  def initialize
    @p = Vector.new
    @q = Quaternion.new
  end

  def set(x, y, z, roll, pitch, yaw)
    @p.set(x, y, z)
    @q.set(roll, pitch, yaw)
  end

  def +(other)
    result = Pose.new

    tmp = Quaternion.new
    tmp.w = 0.0
    tmp.x = other.p.x
    tmp.y = other.p.y
    tmp.z = other.p.z
    tmp = @q * (tmp * @q.inverse)

    result.p.set(@p.x + tmp.x, @p.y + tmp.y, @p.z + tmp.z)
    result.q = other.q * @q

    return result
  end

  def distance(other)
    return @p.distance(other.p)
  end

  attr_accessor :p
  attr_accessor :q
end

class Time
  def initialize
    @sec = 0
    @nsec = 0
  end

  def eql?(other)
    return @sec == other.sec && @nsec == other.nsec
  end

  def hash
    return @sec.to_f + @nsec.to_f * 1e-9
  end

  def -(other)
    result = Time.new
    result.sec = @sec - other.sec
    result.nsec = @nsec - other.nsec
    result.correct
    return result
  end

  def correct
    if (@sec > 0 && @nsec < 0)
      n = (@nsec / 1000000000).abs.to_i + 1
      @sec -= n
      @nsec += n * 1000000000
    end
    if (@sec < 0 && @nsec > 0)
      n = Math.abs(@nsec / 1000000000).to_i + 1
      @sec += n
      @nsec -= n * 1000000000
    end

    @sec += (@nsec / 1000000000).to_i
    @nsec = (@nsec % 1000000000).to_i
  end

  attr_accessor :sec
  attr_accessor :nsec
end

class Color
  def initialize
    @r = 0.0
    @g = 0.0
    @b = 0.0
    @a = 1.0
  end

  def ==(other)
    return @r == other.r && @g == other.g && @b == other.b && @a == other.a
  end

  attr_accessor :r
  attr_accessor :g
  attr_accessor :b
  attr_accessor :a
end

class State
  def initialize(file)
    # Open and read the log file
    doc = Nokogiri::XML(File.open(file))

    # Get the first chunk
    chunks = doc.xpath("//gazebo_log/chunk")
    if chunks.size < 2
      puts "State log file has less than 2 entries."
      return
    end

    @lights = Hash.new

    chunk = Nokogiri::XML(chunks[0].text)

    # Read all the light positions
    for i in 1..44
      light = chunk.xpath("//sdf/world/model/link/visual[@name='light#{i}']")
      lightPoseParts = light.xpath(".//pose").text().split

      @lights[i] = Pose.new

      @lights[i].set(lightPoseParts[0].to_f,
                     lightPoseParts[1].to_f,
                     lightPoseParts[2].to_f,
                     lightPoseParts[3].to_f,
                     lightPoseParts[4].to_f,
                     lightPoseParts[5].to_f)

      # printf("Light[%f %f %f]\n", @lights[i].p.x, @lights[i].p.y,
      #        @lights[i].p.z)
    end

    # Starting model pose
    # @modelPose = Pose.new
    # pose = chunk.xpath("//sdf/world/model[@name='valkyrie']/pose")
    # if pose.size == 1
    #   parts = pose.text().split
    #   @modelPose.set(parts[0].to_f,
    #                  parts[1].to_f,
    #                  parts[2].to_f,
    #                  parts[3].to_f,
    #                  parts[4].to_f,
    #                  parts[5].to_f)
    # end
    # printf("Model[%f %f %f]\n",
    #        @modelPose.p.x, @modelPose.p.y, @modelPose.p.z)


    # Starting head pose
    # @headLinkName = "upperNeckPitchLink"
    # @headPose = Pose.new
    # head = chunk.xpath(
    #       "//sdf/world/model/link[@name='#{@headLinkName}']/pose")
    # if head.size != 1
    #   puts "Error: Unable to find head link[#{upperNeckPitchLink}]!"
    #   return
    # else
    #   parts = head.text().split
    #   @headPose.set(parts[0].to_f,
    #                 parts[1].to_f,
    #                 parts[2].to_f,
    #                 parts[3].to_f,
    #                 parts[4].to_f,
    #                 parts[5].to_f)
    # end
    # printf("Head[%f %f %f]\n", @headPose.p.x, @headPose.p.y, @headPose.p.z)

    # Create hash of the head pose over time
    @poses = Hash.new
    for i in 1..chunks.size-1
      chunk = Nokogiri::XML(chunks[i].text)

      # Read the sim time
      parts = chunk.xpath("//sdf/state/sim_time").text.split
      time = Time.new
      time.sec = parts[0]
      time.nsec = parts[1]

      # Read model pose
      modelPose = Pose.new
      pose = chunk.xpath("//sdf/state/model[@name='valkyrie']/pose")
      if pose.size == 1
        parts = pose.text().split
        modelPose.set(parts[0].to_f, parts[1].to_f, parts[2].to_f,
                      parts[3].to_f, parts[4].to_f, parts[5].to_f)
      end

      # Read the head pose
      headPose = Pose.new
      head = chunk.xpath(
        "//sdf/state/model/link[@name='upperNeckPitchLink']/pose")
      if head.size == 1
        parts = head.text().split
        headPose.set(parts[0].to_f, parts[1].to_f, parts[2].to_f,
                     parts[3].to_f, parts[4].to_f, parts[5].to_f)
      end

      @poses[time] = modelPose + headPose
      # printf("Time[%d.%d] Pose[%f %f %f]\n", time.sec, time.nsec,
      #        @poses[time].p.x, @poses[time].p.y, @poses[time].p.z)
    end
  end

  def lightPose(index)
    pose = Pose.new
    if @lights.has_key?(index)
      pose = @lights[index]
    end
    return pose
  end
end

if ARGV.size != 2
  puts "Usage: scoring.rb <answer.log> <state.log>"
  exit -1
end

qualLog = ARGV[0]
stateLog = ARGV[1]

if !File.file?(qualLog)
  puts "Invalid qual1 log file"
  exit 0
end

if !File.file?(stateLog)
  puts "Invalid state log file"
  exit 0
end

black = Color.new

# Read all the state information
state = State.new(stateLog)

start = Time.new
currentTime = Time.new
lightIndex = -1
error = 0

File.open(qualLog).each do |line|

  # Skip lines that begin with "#"
  if line =~ /^#/
    next
  end

  # Split the line
  parts = line.split

  # Process the "start" line
  if line =~ /^start/
    if parts.size != 3
      puts "Invalid 'start' line, exiting: "
      puts line
      exit 0
    end

    start.sec = parts[1].to_i
    start.nsec = parts[2].to_i
  end

  # Process the "switch" line
  if line =~ /^switch/
    if parts.size != 8
      puts "Invalid 'switch' line, exiting: "
      puts line
      exit 0
    end

    lightColor = Color.new
    lightColor.r = parts[2].to_f
    lightColor.g = parts[3].to_f
    lightColor.b = parts[4].to_f
    lightColor.a = parts[5].to_f

    lightTime = Time.new
    lightTime.sec = parts[6].to_i
    lightTime.nsec = parts[7].to_i
    currentTime = lightTime

    # If not black, then set the light index
    if lightColor != black
      lightIndex = parts[1].to_i
      lightPose = state.lightPose(lightIndex)

       printf("Switch: Time[%4.2f] Color[%2.1f %2.1f %2.1f] Pos[%6.4f %6.4f %6.4f] Index[%d]\n",
              lightTime.sec + lightTime.nsec * 1e-9,
              lightColor.r, lightColor.g, lightColor.b,
              lightPose.p.x, lightPose.p.y, lightPose.p.z,
              lightIndex)
    end
  end

  if line =~ /^answer/
    if parts.size != 9
      puts "Invalid 'answer' line, exiting: "
      puts line
      exit 0
    end

    answerPose = Pose.new
    answerPose.set(parts[1].to_f, parts[2].to_f, parts[3].to_f, 0, 0, 0)

    answerColor = Color.new
    answerColor.r = parts[4].to_f
    answerColor.g = parts[5].to_f
    answerColor.b = parts[6].to_f

    answerTime = Time.new
    answerTime.sec = parts[7].to_i
    answerTime.nsec = parts[8].to_i
    currentTime = answerTime

    # Get the light pose
    lightPose = state.lightPose(lightIndex)

    # Convert the light pose to the head frame

    printf("Answer: Time[%4.2f] Color[%2.1f %2.1f %2.1f] Pos[%6.4f %6.4f %6.4f]\n",
          answerTime.sec + answerTime.nsec * 1e-9,
          answerColor.r, answerColor.g, answerColor.b,
          answerPose.p.x, answerPose.p.y, answerPose.p.z)

    # Compute distance between the light pose and the answer
    error += lightPose.distance(answerPose)
  end
end

duration = Time.new
duration = currentTime - start
printf("Duration: %d.%d Error: %f\n", duration.sec, duration.nsec, error)
