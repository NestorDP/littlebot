## Description
### Problem:
Provide a clear description of the issue

### Goal:
The purpose of this issue is to enhance the codebase by including comprehensive exception tests. These tests will ensure that the system properly handles cases where an invalid or missing IP address is provided, and they will verify that all potential problems during the connection establishment are appropriately handled.

### Proposed Changes:

1. Add test cases to check for invalid or missing IP addresses before starting the UDP connection.
2. Verify that exceptions are raised when attempting to establish a connection without a valid IP address.
3. Test the system's response to various connection-related problems, such as network unavailability or port conflicts, to ensure graceful handling of such scenarios.

### Expected Benefits:
By adding these exceptions tests, we can achieve the following benefits:

Improve the robustness and reliability of the system by catching potential issues during connection establishment.
Reduce the likelihood of unexpected failures and errors, which will facilitate easier debugging and maintenance.
Enhance the overall stability of the system, providing a more seamless experience for end-users.


## Steps to Reproduce
Set up the testing environment to isolate the code related to IP address validation and UDP connection establishment.
Design test cases to cover scenarios where an invalid or missing IP address is provided.
Implement the tests and ensure they raise appropriate exceptions when necessary.
Create test cases to simulate various connection-related problems and validate the system's response.
Run the test suite and observe the results, ensuring that all tests pass successfully.

## Additional Notes
Any additional information or context relevant to the issue can be added here.


