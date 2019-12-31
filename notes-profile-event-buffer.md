12/30/19

Refactor plan
-------------

The ProfileEventBuffer provides an abstraction around the application
event data collected by the ProfileSampler.  When the data is read
from shared memory it is dumped into the ProfileEventBuffer.

Benefits
--------

- Enable multiple users of the application event data through a
  singleton.

- Enable load of ProfileIOGroup when other standard IOGroups are
  loaded.

- Enable usual code path for IOGroup plugin override of application
  signals.  Current implementation does not allow any override of
  application signals.

- Enable easy mocking of application data through more expressive
  access interface.  This should increase test code coverage.


The ProfileSampler class is not modified with this change.  The only
user of the ProfileSampler after the change is the ApplicationIO
class.  The ApplicationIO class is modified by the change so that it
no longer uses the ProfileIOSample class nor the
EpochRuntimeRegulator, and instead uses the ProfileEventBuffer to
insert samples and other data collected from the application.  This
decouples the users of the event data from ApplicationIO.  The
ApplicationIO is still responsible for the hand shake at start-up and
shutdown and gathering all data from the application.  Event data,
rank CPU affinity, profile name, and region name are forward to the
ProfileEventBuffer singleton for other users.  All accounting related
API's, i.e. ApplicationIO::total_*() will be removed by this change.
This data will be tracked by the Reporter::update() and
ProfileIOGroup::read_batch() independently once per Controller loop.


Remove the following classes
----------------------------

    - RuntimeRegulator

    - EpochRuntimeRegulator

    - ProfileIOSample

Effected files current unit test coverage:

                                             hit / total : unhit
    (remove) RuntimeRegulator.cpp:            40 / 43    : 3
    (remove) EpochRuntimeRegulator.cpp:      133 / 192   : 59
    (remove) ProfileIOSample.cpp:              0 / 150   : 150
    (modify) ApplicationIO.cpp:               76 / 166   : 80
    (modify) ProfileIOGroup.cpp:              56 / 231   : 175

The opportunity exists to increase our coverage by around 5-6% overall
with these changes, although the Regulators are pretty well covered.

